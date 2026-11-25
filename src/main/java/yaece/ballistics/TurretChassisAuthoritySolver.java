package yaece.ballistics;

import java.util.Objects;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * High-precision Shoot-on-the-Move (SOTM) turret &amp; chassis angular authority solver addressing
 * three verified field failure modes:
 *
 * <ul>
 *   <li><b>Horizontal floor distance {@code d}:</b> uses 2D floor distance rather than 3D slant
 *       range (which under-leads by ~15% at 4 m).
 *   <li><b>Exact {@code rotateBy(-phi)} frame transform:</b> transforms field-frame robot velocity
 *       into the target-line frame using {@code -phi} so tangential velocity {@code v_perp} has the
 *       exact sign off-axis.
 *   <li><b>{@code 1/d^2} close-range amplification mitigation:</b> splits angular tracking
 *       authority with the swerve chassis via {@code omega_chassis = clamp(-v_perp / max(d, d_min),
 *       -omega_max, +omega_max)}, adds turret muzzle tangential velocity {@code omega_turret x
 *       r_muzzle}, and includes translational-acceleration feedforward.
 * </ul>
 */
public final class TurretChassisAuthoritySolver {

  /**
   * Computed SOTM turret/chassis aiming solution.
   *
   * @param horizontalDistanceMeters 2D horizontal floor distance from muzzle to target in meters.
   * @param radialVelocityMps Velocity component closing toward (+) or away from (-) the target in
   *     m/s.
   * @param tangentialVelocityMps Velocity component perpendicular to the target line (+left) in
   *     m/s.
   * @param fieldAimYawRad Required field-frame aim yaw in radians (velocity- and accel-lead
   *     compensated).
   * @param totalRequiredOmegaRadPerSec Total angular velocity required to track the target in
   *     rad/s.
   * @param chassisOmegaFeedforwardRadPerSec Clamped chassis angular velocity share in rad/s.
   * @param turretOmegaFeedforwardRadPerSec Residual turret angular velocity feedforward in rad/s.
   * @param virtualTargetMeters Velocity- and acceleration-compensated 2D virtual target point in
   *     meters.
   */
  public record AuthoritySolution(
      double horizontalDistanceMeters,
      double radialVelocityMps,
      double tangentialVelocityMps,
      double fieldAimYawRad,
      double totalRequiredOmegaRadPerSec,
      double chassisOmegaFeedforwardRadPerSec,
      double turretOmegaFeedforwardRadPerSec,
      Translation2d virtualTargetMeters) {}

  private final double minEffectiveDistanceMeters;
  private final double maxChassisOmegaRadPerSec;
  private final double chassisShareFraction;

  /**
   * Creates a turret-chassis authority solver.
   *
   * @param minEffectiveDistanceMeters Distance floor {@code d_min} in meters preventing {@code 1/d}
   *     blow-up at close range (typically {@code 1.25 m}).
   * @param maxChassisOmegaRadPerSec Symmetric clamp on chassis angular velocity feedforward in
   *     rad/s (typically {@code 2.0 rad/s}).
   * @param chassisShareFraction Fraction of required angular velocity assigned to the chassis in
   *     {@code [0.0, 1.0]} (for example {@code 0.5} for 50/50 turret/chassis sharing, {@code 1.0}
   *     for turretless swerve).
   */
  public TurretChassisAuthoritySolver(
      double minEffectiveDistanceMeters,
      double maxChassisOmegaRadPerSec,
      double chassisShareFraction) {
    if (minEffectiveDistanceMeters <= 0.0 || !Double.isFinite(minEffectiveDistanceMeters)) {
      throw new IllegalArgumentException("minEffectiveDistanceMeters must be positive.");
    }
    if (maxChassisOmegaRadPerSec < 0.0 || !Double.isFinite(maxChassisOmegaRadPerSec)) {
      throw new IllegalArgumentException("maxChassisOmegaRadPerSec must be non-negative.");
    }
    if (chassisShareFraction < 0.0 || chassisShareFraction > 1.0) {
      throw new IllegalArgumentException("chassisShareFraction must be in [0.0, 1.0].");
    }
    this.minEffectiveDistanceMeters = minEffectiveDistanceMeters;
    this.maxChassisOmegaRadPerSec = maxChassisOmegaRadPerSec;
    this.chassisShareFraction = chassisShareFraction;
  }

  /**
   * Solves velocity- and acceleration-compensated SOTM aim yaw and turret/chassis angular velocity
   * feedforward.
   *
   * @param robotPosMeters Field-frame robot center X/Y in meters.
   * @param robotVelMps Field-frame robot linear velocity in m/s.
   * @param robotAccelMps2 Field-frame robot linear acceleration in m/s^2 (pass zero if
   *     unavailable).
   * @param robotOmegaRadPerSec Current chassis yaw rate in rad/s.
   * @param muzzleOffsetFieldMeters Field-frame vector from robot center to turret muzzle in meters.
   * @param turretOmegaRadPerSec Current turret angular velocity relative to chassis in rad/s.
   * @param targetPosMeters Field-frame 2D target center in meters.
   * @param timeOfFlightSeconds Predicted horizontal time of flight in seconds.
   * @return Computed {@link AuthoritySolution}.
   */
  public AuthoritySolution solve(
      Translation2d robotPosMeters,
      Translation2d robotVelMps,
      Translation2d robotAccelMps2,
      double robotOmegaRadPerSec,
      Translation2d muzzleOffsetFieldMeters,
      double turretOmegaRadPerSec,
      Translation2d targetPosMeters,
      double timeOfFlightSeconds) {
    Objects.requireNonNull(robotPosMeters, "robotPosMeters");
    Objects.requireNonNull(robotVelMps, "robotVelMps");
    Objects.requireNonNull(robotAccelMps2, "robotAccelMps2");
    Objects.requireNonNull(muzzleOffsetFieldMeters, "muzzleOffsetFieldMeters");
    Objects.requireNonNull(targetPosMeters, "targetPosMeters");
    double tof = Math.max(0.0, timeOfFlightSeconds);

    // Muzzle field position & tangential velocity from (omega_chassis + omega_turret) x r_muzzle
    Translation2d muzzlePos = robotPosMeters.plus(muzzleOffsetFieldMeters);
    double totalSpin = robotOmegaRadPerSec + turretOmegaRadPerSec;
    double muzzleTanVx = -totalSpin * muzzleOffsetFieldMeters.getY();
    double muzzleTanVy = totalSpin * muzzleOffsetFieldMeters.getX();

    double effVx = robotVelMps.getX() + muzzleTanVx;
    double effVy = robotVelMps.getY() + muzzleTanVy;

    // 2D horizontal vector from muzzle to target (never 3D slant range)
    Translation2d toTarget = targetPosMeters.minus(muzzlePos);
    double d = toTarget.getNorm();
    double phi = Math.atan2(toTarget.getY(), toTarget.getX());

    // Exact rotateBy(-phi) transforms field velocity into target-line frame (+X radial, +Y left)
    Translation2d velInTargetFrame = new Translation2d(effVx, effVy).rotateBy(new Rotation2d(-phi));
    double vRadial = velInTargetFrame.getX();
    double vPerp = velInTargetFrame.getY();

    // Virtual target compensation including velocity + 0.5 * a * t^2 translational accel FF
    double virtualX =
        targetPosMeters.getX() - effVx * tof - 0.5 * robotAccelMps2.getX() * tof * tof;
    double virtualY =
        targetPosMeters.getY() - effVy * tof - 0.5 * robotAccelMps2.getY() * tof * tof;
    Translation2d virtualTarget = new Translation2d(virtualX, virtualY);
    double aimYaw = Math.atan2(virtualY - muzzlePos.getY(), virtualX - muzzlePos.getX());

    // Analytic angular velocity FF: omega = -v_perp / max(d, d_min)
    double safeD = Math.max(d, minEffectiveDistanceMeters);
    double totalRequiredOmega = -vPerp / safeD;
    double chassisOmega =
        clamp(
            totalRequiredOmega * chassisShareFraction,
            -maxChassisOmegaRadPerSec,
            maxChassisOmegaRadPerSec);
    double turretOmega = totalRequiredOmega - chassisOmega;

    return new AuthoritySolution(
        d, vRadial, vPerp, aimYaw, totalRequiredOmega, chassisOmega, turretOmega, virtualTarget);
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
