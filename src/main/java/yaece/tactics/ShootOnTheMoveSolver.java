// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.season.FieldMirroring;

/**
 * Game-agnostic Shoot-on-the-Move (SOTM) iterative solver combining SE(2) phase-delay lookahead,
 * rigid-body launcher-offset velocity kinematics, exponential aerodynamic drag compensation,
 * analytical aiming angular-velocity feedforward, and geometric exclusion zones ("box of bad").
 */
public final class ShootOnTheMoveSolver {

  /**
   * Immutable configuration for the Shoot-on-the-Move solver.
   *
   * @param phaseDelaySec actuator and latency lookahead in seconds (e.g. 0.03 s)
   * @param linearDragCoeff linear drag constant {@code k} in {@code (1 - exp(-tof * k)) / k}; 0.0
   *     disables drag (vacuum model)
   * @param maxIterations maximum fixed-point iterations (typically 20)
   * @param launcherOffsetRobot launcher exit offset in robot frame (meters)
   * @param minRangeMeters minimum valid effective shooting distance (meters)
   * @param maxRangeMeters maximum valid effective shooting distance (meters)
   */
  public record Config(
      double phaseDelaySec,
      double linearDragCoeff,
      int maxIterations,
      Translation2d launcherOffsetRobot,
      double minRangeMeters,
      double maxRangeMeters) {

    public static final Config REBUILT_DEFAULT =
        new Config(0.03, 0.375, 20, new Translation2d(), 1.2, 8.5);

    public Config {
      if (phaseDelaySec < 0.0) {
        throw new IllegalArgumentException("phaseDelaySec must be >= 0");
      }
      if (linearDragCoeff < 0.0) {
        throw new IllegalArgumentException("linearDragCoeff must be >= 0");
      }
      if (maxIterations < 1) {
        throw new IllegalArgumentException("maxIterations must be >= 1");
      }
      Objects.requireNonNull(launcherOffsetRobot, "launcherOffsetRobot must not be null");
      if (minRangeMeters < 0.0 || maxRangeMeters < minRangeMeters) {
        throw new IllegalArgumentException("Invalid range bounds");
      }
    }
  }

  /**
   * Solved Shoot-on-the-Move aiming and range solution.
   *
   * @param virtualTargetField lead-compensated virtual target in field coordinates
   * @param effectiveDistanceMeters horizontal distance from predicted launcher to virtual target
   * @param rawTofSec unattenuated lookup time-of-flight at the converged distance
   * @param effectiveTofSec drag-attenuated effective time-of-flight used for lead calculation
   * @param aimHeadingField field-relative heading from predicted launcher to virtual target
   * @param aimOmegaFfRadPerSec analytical angular-velocity feedforward (rad/s) to track the virtual
   *     target while moving
   * @param inRange true when {@code effectiveDistanceMeters} lies within {@code [minRange,
   *     maxRange]}
   * @param insideExclusionZone true when the robot or predicted pose lies inside any configured
   *     exclusion polygon
   * @param iterations number of fixed-point iterations executed
   */
  public record SotmSolution(
      Translation3d virtualTargetField,
      double effectiveDistanceMeters,
      double rawTofSec,
      double effectiveTofSec,
      Rotation2d aimHeadingField,
      double aimOmegaFfRadPerSec,
      boolean inRange,
      boolean insideExclusionZone,
      int iterations) {}

  private static final double CONVERGENCE_EPSILON_M = 1e-4;
  private static final double MIN_RADIUS_SQ = 1e-6;

  private final Config config;
  private final DoubleUnaryOperator tofLookupByDistanceMeters;
  private final List<List<Translation2d>> exclusionZonesField;

  /** Creates a solver with no geometric exclusion zones. */
  public ShootOnTheMoveSolver(Config config, DoubleUnaryOperator tofLookupByDistanceMeters) {
    this(config, tofLookupByDistanceMeters, List.of());
  }

  /** Creates a solver with optional field-space exclusion polygons. */
  public ShootOnTheMoveSolver(
      Config config,
      DoubleUnaryOperator tofLookupByDistanceMeters,
      List<List<Translation2d>> exclusionZonesField) {
    this.config = Objects.requireNonNull(config, "config must not be null");
    this.tofLookupByDistanceMeters =
        Objects.requireNonNull(
            tofLookupByDistanceMeters, "tofLookupByDistanceMeters must not be null");
    Objects.requireNonNull(exclusionZonesField, "exclusionZonesField must not be null");
    List<List<Translation2d>> copy = new ArrayList<>(exclusionZonesField.size());
    for (List<Translation2d> poly : exclusionZonesField) {
      copy.add(List.copyOf(poly));
    }
    this.exclusionZonesField = List.copyOf(copy);
  }

  /**
   * Solves for the lead-compensated virtual target, drag-attenuated time-of-flight, aim heading,
   * and analytical angular-velocity feedforward.
   *
   * @param egoPoseField current field-relative robot pose
   * @param vxFieldMps current field-relative robot X velocity (m/s)
   * @param vyFieldMps current field-relative robot Y velocity (m/s)
   * @param omegaRadPerSec current robot angular velocity (rad/s)
   * @param targetField stationary scoring target position in field coordinates
   * @return converged {@link SotmSolution}
   */
  public SotmSolution solve(
      Pose2d egoPoseField,
      double vxFieldMps,
      double vyFieldMps,
      double omegaRadPerSec,
      Translation3d targetField) {
    Objects.requireNonNull(egoPoseField, "egoPoseField must not be null");
    Objects.requireNonNull(targetField, "targetField must not be null");

    // 1. SE(2) phase-delay pose prediction
    Pose2d predictedEgo = predictPoseSe2(egoPoseField, vxFieldMps, vyFieldMps, omegaRadPerSec);

    // 2. Launcher position and rigid-body velocity in field frame: v_L = v_robot + omega x r_offset
    Translation2d launcherOffsetField =
        config.launcherOffsetRobot().rotateBy(predictedEgo.getRotation());
    Translation2d launcherPosField = predictedEgo.getTranslation().plus(launcherOffsetField);
    double launcherVx = vxFieldMps - omegaRadPerSec * launcherOffsetField.getY();
    double launcherVy = vyFieldMps + omegaRadPerSec * launcherOffsetField.getX();

    // 3. Fixed-point iteration with exponential drag effective time-of-flight
    double virtualX = targetField.getX();
    double virtualY = targetField.getY();
    double rawTof = 0.0;
    double effectiveTof = 0.0;
    int iterations = 0;

    for (int i = 0; i < config.maxIterations(); i++) {
      iterations = i + 1;
      double dist =
          Math.hypot(virtualX - launcherPosField.getX(), virtualY - launcherPosField.getY());
      rawTof = Math.max(0.0, tofLookupByDistanceMeters.applyAsDouble(dist));
      effectiveTof = computeEffectiveTof(rawTof, config.linearDragCoeff());

      double nextX = targetField.getX() - launcherVx * effectiveTof;
      double nextY = targetField.getY() - launcherVy * effectiveTof;
      double delta = Math.hypot(nextX - virtualX, nextY - virtualY);
      virtualX = nextX;
      virtualY = nextY;
      if (delta < CONVERGENCE_EPSILON_M) {
        break;
      }
    }

    double rx = virtualX - launcherPosField.getX();
    double ry = virtualY - launcherPosField.getY();
    double effectiveDist = Math.hypot(rx, ry);
    Rotation2d aimHeading =
        effectiveDist > 1e-6 ? new Rotation2d(rx, ry) : predictedEgo.getRotation();

    // 4. Analytical angular-velocity feedforward: d/dt atan2(ry, rx) = (ry * vx_L - rx * vy_L) /
    // r^2
    double rSq = rx * rx + ry * ry;
    double aimOmegaFf = rSq > MIN_RADIUS_SQ ? (ry * launcherVx - rx * launcherVy) / rSq : 0.0;

    // 5. Range and geometric exclusion-zone checks
    boolean inRange =
        effectiveDist >= config.minRangeMeters() && effectiveDist <= config.maxRangeMeters();
    boolean insideExclusion =
        isInsideAnyExclusionZone(egoPoseField.getTranslation())
            || isInsideAnyExclusionZone(predictedEgo.getTranslation());

    return new SotmSolution(
        new Translation3d(virtualX, virtualY, targetField.getZ()),
        effectiveDist,
        rawTof,
        effectiveTof,
        aimHeading,
        aimOmegaFf,
        inRange,
        insideExclusion,
        iterations);
  }

  private Pose2d predictPoseSe2(
      Pose2d egoPoseField, double vxFieldMps, double vyFieldMps, double omegaRadPerSec) {
    double dt = config.phaseDelaySec();
    if (dt <= 0.0) {
      return egoPoseField;
    }
    Translation2d robotVel =
        new Translation2d(vxFieldMps, vyFieldMps).rotateBy(egoPoseField.getRotation().unaryMinus());
    double dx = robotVel.getX() * dt;
    double dy = robotVel.getY() * dt;
    double dtheta = omegaRadPerSec * dt;
    double s;
    double c;
    if (Math.abs(dtheta) < 1e-9) {
      s = 1.0 - (1.0 / 6.0) * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = Math.sin(dtheta) / dtheta;
      c = (1.0 - Math.cos(dtheta)) / dtheta;
    }
    Translation2d localDelta = new Translation2d(dx * s - dy * c, dx * c + dy * s);
    return new Pose2d(
        egoPoseField.getTranslation().plus(localDelta.rotateBy(egoPoseField.getRotation())),
        egoPoseField.getRotation().plus(Rotation2d.fromRadians(dtheta)));
  }

  private static double computeEffectiveTof(double rawTofSec, double linearDragCoeff) {
    if (linearDragCoeff <= 1e-6 || rawTofSec <= 0.0) {
      return rawTofSec;
    }
    return (1.0 - Math.exp(-rawTofSec * linearDragCoeff)) / linearDragCoeff;
  }

  private boolean isInsideAnyExclusionZone(Translation2d pointField) {
    for (List<Translation2d> poly : exclusionZonesField) {
      if (FieldMirroring.polygonContainsPoint(poly, pointField)) {
        return true;
      }
    }
    return false;
  }
}
