// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;

/**
 * Dual-gated shot and scoring readiness evaluator combining FRC 1678's {@code ShotVerifier}, FRC
 * 2910's {@code isReadyToScore}, and FRC 3847's hysteresis volley gate (tighter entry tolerances,
 * relaxed keep-feeding tolerances, settle hold timer, hard timeout, operator bypass, and pose-free
 * SetShot fallback).
 */
public final class ShotReadinessGate {
  public record Config(
      double maxPositionErrorMeters,
      double maxHeadingErrorDeg,
      double maxTiltDeg,
      double maxChassisSpeedMps,
      double maxVisionAgeSec,
      double autoMaxWaitSec,
      boolean requireFreshVisionInTeleop) {
    public static final Config FRC_DEFAULT = new Config(0.08, 3.0, 5.0, 0.20, 0.10, 0.45, true);
  }

  /** Hysteresis &amp; override parameters for continuous volley feeding (FRC 3847 pattern). */
  public record VolleyConfig(
      double settleHoldSec,
      double hardTimeoutSec,
      double keepFeedingHeadingMultiplier,
      double keepFeedingPositionMultiplier) {
    public static final VolleyConfig FRC_DEFAULT = new VolleyConfig(0.04, 1.0, 2.0, 1.5);
  }

  /**
   * Distance-scaled aim tolerance and post-bump hold parameters modeled on FRC 3847's {@code
   * ShotGate} and {@code TiltState}.
   *
   * @param minHeadingToleranceDeg Tightest aim tolerance at far range in degrees (default 2.0).
   * @param maxHeadingToleranceDeg Widest aim tolerance at close range in degrees (default 10.0).
   * @param closeRangeMeters Distance at or below which {@code maxHeadingToleranceDeg} applies.
   * @param farRangeMeters Distance at or above which {@code minHeadingToleranceDeg} applies.
   * @param bumpAccelThresholdG Acceleration shock threshold in g's that latches a post-bump hold
   *     (default 0.5 g).
   * @param postBumpHoldSec Duration in seconds to hold firing after a bump shock (default 0.5 s).
   * @param autoMaxHoldTimeoutSec Maximum wait duration in autonomous before fail-safe release
   *     overrides bump/vision holds so auto never deadlocks (default 0.45 s).
   */
  public record DynamicGateConfig(
      double minHeadingToleranceDeg,
      double maxHeadingToleranceDeg,
      double closeRangeMeters,
      double farRangeMeters,
      double bumpAccelThresholdG,
      double postBumpHoldSec,
      double autoMaxHoldTimeoutSec) {

    /** FRC 3847-inspired default dynamic shot gate parameters. */
    public static final DynamicGateConfig FRC_DEFAULT =
        new DynamicGateConfig(2.0, 10.0, 1.5, 6.0, 0.5, 0.5, 0.45);

    /** Compact constructor validating dynamic gate bounds. */
    public DynamicGateConfig {
      if (!Double.isFinite(minHeadingToleranceDeg) || minHeadingToleranceDeg <= 0.0) {
        throw new IllegalArgumentException("minHeadingToleranceDeg must be finite and > 0");
      }
      if (!Double.isFinite(maxHeadingToleranceDeg)
          || maxHeadingToleranceDeg < minHeadingToleranceDeg) {
        throw new IllegalArgumentException(
            "maxHeadingToleranceDeg must be finite and >= minHeadingToleranceDeg");
      }
      if (!Double.isFinite(closeRangeMeters) || closeRangeMeters < 0.0) {
        throw new IllegalArgumentException("closeRangeMeters must be finite and >= 0");
      }
      if (!Double.isFinite(farRangeMeters) || farRangeMeters <= closeRangeMeters) {
        throw new IllegalArgumentException("farRangeMeters must be finite and > closeRangeMeters");
      }
      if (!Double.isFinite(bumpAccelThresholdG) || bumpAccelThresholdG <= 0.0) {
        throw new IllegalArgumentException("bumpAccelThresholdG must be finite and > 0");
      }
      if (!Double.isFinite(postBumpHoldSec) || postBumpHoldSec < 0.0) {
        throw new IllegalArgumentException("postBumpHoldSec must be finite and >= 0");
      }
      if (!Double.isFinite(autoMaxHoldTimeoutSec) || autoMaxHoldTimeoutSec < 0.0) {
        throw new IllegalArgumentException("autoMaxHoldTimeoutSec must be finite and >= 0");
      }
    }
  }

  public record ReadinessResult(
      boolean readyToScore, boolean driverRumbleVisionStale, String reason) {}

  private final Config config;
  private final VolleyConfig volleyConfig;
  private final DynamicGateConfig dynamicConfig;
  private double settleStartSec = -1.0;
  private boolean volleyActive = false;
  private double lastBumpSec = -1.0;

  public ShotReadinessGate() {
    this(Config.FRC_DEFAULT, VolleyConfig.FRC_DEFAULT, DynamicGateConfig.FRC_DEFAULT);
  }

  public ShotReadinessGate(Config config) {
    this(config, VolleyConfig.FRC_DEFAULT, DynamicGateConfig.FRC_DEFAULT);
  }

  public ShotReadinessGate(Config config, VolleyConfig volleyConfig) {
    this(config, volleyConfig, DynamicGateConfig.FRC_DEFAULT);
  }

  /**
   * Creates a {@link ShotReadinessGate} with explicit base, volley, and dynamic gate configs.
   *
   * @param config Base geometric/vision readiness tolerances.
   * @param volleyConfig Continuous volley hysteresis parameters.
   * @param dynamicConfig Distance-scaled aim tolerance and post-bump hold parameters.
   */
  public ShotReadinessGate(
      Config config, VolleyConfig volleyConfig, DynamicGateConfig dynamicConfig) {
    this.config = Objects.requireNonNull(config, "config must not be null");
    this.volleyConfig = Objects.requireNonNull(volleyConfig, "volleyConfig must not be null");
    this.dynamicConfig = Objects.requireNonNull(dynamicConfig, "dynamicConfig must not be null");
  }

  /** Resets volley hysteresis, settle hold, and post-bump hold timers. */
  public void reset() {
    settleStartSec = -1.0;
    volleyActive = false;
    lastBumpSec = -1.0;
  }

  /**
   * Records an IMU linear acceleration shock magnitude (in g's above 1 g gravity baseline). When
   * {@code Math.abs(accelDeltaG) > dynamicConfig.bumpAccelThresholdG()}, latches a deterministic
   * post-bump hold for {@code dynamicConfig.postBumpHoldSec()} seconds.
   *
   * @param accelDeltaG Linear shock magnitude in g's.
   * @param nowSec Deterministic robot/replay timestamp in seconds.
   */
  public void recordChassisAccelerationG(double accelDeltaG, double nowSec) {
    if (Double.isFinite(accelDeltaG)
        && Double.isFinite(nowSec)
        && Math.abs(accelDeltaG) > dynamicConfig.bumpAccelThresholdG()) {
      lastBumpSec = nowSec;
    }
  }

  /**
   * Returns true while the chassis is inside its post-bump settle hold window at {@code nowSec}.
   *
   * @param nowSec Deterministic robot/replay timestamp in seconds.
   * @return True when a recent bump shock is still holding shots.
   */
  public boolean isInPostBumpHold(double nowSec) {
    return lastBumpSec >= 0.0
        && nowSec >= lastBumpSec
        && (nowSec - lastBumpSec) < dynamicConfig.postBumpHoldSec();
  }

  /**
   * Computes the distance-scaled heading tolerance in degrees, linearly interpolating between
   * {@code maxHeadingToleranceDeg} at {@code closeRangeMeters} and {@code minHeadingToleranceDeg}
   * at {@code farRangeMeters}.
   *
   * @param targetDistanceMeters Planar distance to the scoring target in meters.
   * @param cfg Dynamic gate configuration.
   * @return Clamped heading tolerance in degrees.
   */
  public static double distanceScaledHeadingToleranceDeg(
      double targetDistanceMeters, DynamicGateConfig cfg) {
    Objects.requireNonNull(cfg, "cfg must not be null");
    if (!Double.isFinite(targetDistanceMeters) || targetDistanceMeters <= cfg.closeRangeMeters()) {
      return cfg.maxHeadingToleranceDeg();
    }
    if (targetDistanceMeters >= cfg.farRangeMeters()) {
      return cfg.minHeadingToleranceDeg();
    }
    double t =
        (targetDistanceMeters - cfg.closeRangeMeters())
            / (cfg.farRangeMeters() - cfg.closeRangeMeters());
    return cfg.maxHeadingToleranceDeg()
        + t * (cfg.minHeadingToleranceDeg() - cfg.maxHeadingToleranceDeg());
  }

  public ReadinessResult evaluate(
      Pose3d egoPose3d,
      Pose3d targetPose3d,
      double chassisSpeedMps,
      boolean mechanismReady,
      double nowSec,
      double lastVisionUpdateSec,
      boolean isAutonomous,
      double waitElapsedSec) {
    if (!mechanismReady) {
      return new ReadinessResult(false, false, "Mechanism not ready");
    }

    double tiltDeg =
        Math.toDegrees(Math.hypot(egoPose3d.getRotation().getX(), egoPose3d.getRotation().getY()));
    if (tiltDeg > config.maxTiltDeg()) {
      return new ReadinessResult(
          false,
          false,
          String.format("Chassis tilted (%.1f deg > %.1f deg)", tiltDeg, config.maxTiltDeg()));
    }

    if (chassisSpeedMps > config.maxChassisSpeedMps()) {
      return new ReadinessResult(
          false,
          false,
          String.format(
              "Chassis speed %.2f m/s exceeds %.2f m/s",
              chassisSpeedMps, config.maxChassisSpeedMps()));
    }

    double posErr =
        egoPose3d
            .getTranslation()
            .toTranslation2d()
            .getDistance(targetPose3d.getTranslation().toTranslation2d());
    if (posErr > config.maxPositionErrorMeters()) {
      return new ReadinessResult(
          false,
          false,
          String.format("Position error %.2f m > %.2f m", posErr, config.maxPositionErrorMeters()));
    }

    double headingErrDeg =
        Math.toDegrees(
            Math.abs(wrapRad(targetPose3d.getRotation().getZ() - egoPose3d.getRotation().getZ())));
    if (headingErrDeg > config.maxHeadingErrorDeg()) {
      return new ReadinessResult(
          false,
          false,
          String.format(
              "Heading error %.1f deg > %.1f deg", headingErrDeg, config.maxHeadingErrorDeg()));
    }

    boolean visionFresh =
        lastVisionUpdateSec >= 0.0 && (nowSec - lastVisionUpdateSec) <= config.maxVisionAgeSec();
    if (!visionFresh) {
      if (isAutonomous && waitElapsedSec >= config.autoMaxWaitSec()) {
        return new ReadinessResult(
            true, false, "Auto fail-safe timeout reached; releasing on odometry");
      }
      if (!isAutonomous && !config.requireFreshVisionInTeleop()) {
        return new ReadinessResult(true, false, "Ready (teleop vision check disabled)");
      }
      return new ReadinessResult(
          false, !isAutonomous, "Odometry converged; waiting on fresh vision");
    }

    return new ReadinessResult(true, false, "All readiness gates satisfied");
  }

  /**
   * Hysteresis-aware volley evaluation supporting settle-hold debouncing, relaxed keep-feeding
   * tolerances once a volley starts, hard timeout release, operator bypass, and pose-free SetShot
   * fallback (when vision/odometry is degraded).
   */
  public ReadinessResult evaluateVolley(
      Pose3d egoPose3d,
      Pose3d targetPose3d,
      double chassisSpeedMps,
      boolean mechanismReady,
      double nowSec,
      double lastVisionUpdateSec,
      boolean isAutonomous,
      double waitElapsedSec,
      boolean operatorBypass,
      boolean poseFreeSetShot) {
    if (operatorBypass) {
      volleyActive = true;
      return new ReadinessResult(true, false, "Operator bypass active");
    }
    if (!mechanismReady) {
      reset();
      return new ReadinessResult(false, false, "Mechanism not ready");
    }
    if (poseFreeSetShot) {
      volleyActive = true;
      return new ReadinessResult(true, false, "Pose-free SetShot fallback active");
    }

    double posLimit =
        volleyActive
            ? config.maxPositionErrorMeters() * volleyConfig.keepFeedingPositionMultiplier()
            : config.maxPositionErrorMeters();
    double headingLimit =
        volleyActive
            ? config.maxHeadingErrorDeg() * volleyConfig.keepFeedingHeadingMultiplier()
            : config.maxHeadingErrorDeg();

    double tiltDeg =
        Math.toDegrees(Math.hypot(egoPose3d.getRotation().getX(), egoPose3d.getRotation().getY()));
    double posErr =
        egoPose3d
            .getTranslation()
            .toTranslation2d()
            .getDistance(targetPose3d.getTranslation().toTranslation2d());
    double headingErrDeg =
        Math.toDegrees(
            Math.abs(wrapRad(targetPose3d.getRotation().getZ() - egoPose3d.getRotation().getZ())));

    boolean withinGeometry =
        tiltDeg <= config.maxTiltDeg()
            && chassisSpeedMps <= config.maxChassisSpeedMps()
            && posErr <= posLimit
            && headingErrDeg <= headingLimit;

    if (!withinGeometry) {
      if (waitElapsedSec >= volleyConfig.hardTimeoutSec()) {
        volleyActive = true;
        return new ReadinessResult(true, false, "Hard volley timeout reached; releasing");
      }
      reset();
      return new ReadinessResult(
          false,
          false,
          String.format("Outside volley gate (pos=%.2fm, hdg=%.1fdeg)", posErr, headingErrDeg));
    }

    boolean visionFresh =
        lastVisionUpdateSec >= 0.0 && (nowSec - lastVisionUpdateSec) <= config.maxVisionAgeSec();
    if (!visionFresh && !(isAutonomous && waitElapsedSec >= config.autoMaxWaitSec())) {
      if (waitElapsedSec >= volleyConfig.hardTimeoutSec()) {
        volleyActive = true;
        return new ReadinessResult(true, true, "Hard volley timeout reached with stale vision");
      }
      return new ReadinessResult(false, !isAutonomous, "Waiting on fresh vision for volley");
    }

    if (!volleyActive) {
      if (settleStartSec < 0.0) {
        settleStartSec = nowSec;
      }
      if ((nowSec - settleStartSec) < volleyConfig.settleHoldSec()) {
        return new ReadinessResult(false, false, "Settling within volley gate");
      }
      volleyActive = true;
    }
    return new ReadinessResult(true, false, "Volley readiness gate satisfied");
  }

  /**
   * Deterministic, replay-safe volley evaluation combining distance-scaled heading tolerance
   * ({@code [2 deg, 10 deg]} default), post-bump hold ({@code 0.5 s} after {@code > 0.5 g}), and
   * autonomous max-hold fail-safe timeout so auto routines never deadlock.
   *
   * @param egoPose3d Current robot 3D pose.
   * @param targetPose3d Desired scoring pose/heading.
   * @param targetDistanceMeters Planar distance to the scoring target in meters.
   * @param chassisSpeedMps Current planar chassis speed in m/s.
   * @param mechanismReady Whether shooter/mechanism is at setpoint.
   * @param nowSec Deterministic robot/replay timestamp in seconds.
   * @param lastVisionUpdateSec Timestamp of last accepted vision pose update.
   * @param isAutonomous True during autonomous period.
   * @param waitElapsedSec Time spent waiting on this shot in seconds.
   * @param operatorBypass True when operator manually overrides gates.
   * @param poseFreeSetShot True when using fixed-distance pose-free SetShot fallback.
   * @return {@link ReadinessResult} indicating whether to fire and diagnostic reason.
   */
  public ReadinessResult evaluateDynamic(
      Pose3d egoPose3d,
      Pose3d targetPose3d,
      double targetDistanceMeters,
      double chassisSpeedMps,
      boolean mechanismReady,
      double nowSec,
      double lastVisionUpdateSec,
      boolean isAutonomous,
      double waitElapsedSec,
      boolean operatorBypass,
      boolean poseFreeSetShot) {
    if (operatorBypass) {
      volleyActive = true;
      return new ReadinessResult(true, false, "Operator bypass active");
    }
    if (!mechanismReady) {
      settleStartSec = -1.0;
      volleyActive = false;
      return new ReadinessResult(false, false, "Mechanism not ready");
    }
    if (poseFreeSetShot) {
      volleyActive = true;
      return new ReadinessResult(true, false, "Pose-free SetShot fallback active");
    }

    if (isInPostBumpHold(nowSec)) {
      if (isAutonomous && waitElapsedSec >= dynamicConfig.autoMaxHoldTimeoutSec()) {
        volleyActive = true;
        return new ReadinessResult(
            true, false, "Auto max-hold timeout overrode post-bump hold; releasing");
      }
      settleStartSec = -1.0;
      volleyActive = false;
      return new ReadinessResult(
          false,
          false,
          String.format(
              "Post-bump hold active (%.2fs remaining)",
              dynamicConfig.postBumpHoldSec() - (nowSec - lastBumpSec)));
    }

    double baseHeadingDeg = distanceScaledHeadingToleranceDeg(targetDistanceMeters, dynamicConfig);
    double posLimit =
        volleyActive
            ? config.maxPositionErrorMeters() * volleyConfig.keepFeedingPositionMultiplier()
            : config.maxPositionErrorMeters();
    double headingLimit =
        volleyActive
            ? baseHeadingDeg * volleyConfig.keepFeedingHeadingMultiplier()
            : baseHeadingDeg;

    double tiltDeg =
        Math.toDegrees(Math.hypot(egoPose3d.getRotation().getX(), egoPose3d.getRotation().getY()));
    double posErr =
        egoPose3d
            .getTranslation()
            .toTranslation2d()
            .getDistance(targetPose3d.getTranslation().toTranslation2d());
    double headingErrDeg =
        Math.toDegrees(
            Math.abs(wrapRad(targetPose3d.getRotation().getZ() - egoPose3d.getRotation().getZ())));

    boolean withinGeometry =
        tiltDeg <= config.maxTiltDeg()
            && chassisSpeedMps <= config.maxChassisSpeedMps()
            && posErr <= posLimit
            && headingErrDeg <= headingLimit;

    if (!withinGeometry) {
      double timeoutSec =
          isAutonomous
              ? Math.min(volleyConfig.hardTimeoutSec(), dynamicConfig.autoMaxHoldTimeoutSec())
              : volleyConfig.hardTimeoutSec();
      if (waitElapsedSec >= timeoutSec) {
        volleyActive = true;
        return new ReadinessResult(
            true,
            false,
            isAutonomous
                ? "Auto max-hold timeout reached; releasing"
                : "Hard volley timeout reached; releasing");
      }
      settleStartSec = -1.0;
      volleyActive = false;
      return new ReadinessResult(
          false,
          false,
          String.format(
              "Outside dynamic gate (pos=%.2fm, hdg=%.1fdeg > %.1fdeg)",
              posErr, headingErrDeg, headingLimit));
    }

    boolean visionFresh =
        lastVisionUpdateSec >= 0.0 && (nowSec - lastVisionUpdateSec) <= config.maxVisionAgeSec();
    double autoTimeout = Math.min(config.autoMaxWaitSec(), dynamicConfig.autoMaxHoldTimeoutSec());
    if (!visionFresh && !(isAutonomous && waitElapsedSec >= autoTimeout)) {
      if (waitElapsedSec >= volleyConfig.hardTimeoutSec()) {
        volleyActive = true;
        return new ReadinessResult(true, true, "Hard volley timeout reached with stale vision");
      }
      return new ReadinessResult(false, !isAutonomous, "Waiting on fresh vision for dynamic shot");
    }

    if (!volleyActive) {
      if (settleStartSec < 0.0) {
        settleStartSec = nowSec;
      }
      if ((nowSec - settleStartSec) < volleyConfig.settleHoldSec()) {
        return new ReadinessResult(false, false, "Settling within dynamic shot gate");
      }
      volleyActive = true;
    }
    return new ReadinessResult(true, false, "Dynamic shot readiness gate satisfied");
  }

  private static double wrapRad(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
