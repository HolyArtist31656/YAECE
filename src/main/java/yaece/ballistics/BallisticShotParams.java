package yaece.ballistics;

/**
 * Immutable interpolated shooter setpoint parameters for a given horizontal distance to target.
 *
 * @param flywheelRps target flywheel surface or shaft speed in rotations per second
 * @param hoodAngleRad target hood elevation angle in radians
 * @param timeOfFlightSec empirical projectile time-of-flight in seconds
 * @param headingToleranceRad distance-dependent allowable aiming heading error in radians
 */
public record BallisticShotParams(
    double flywheelRps, double hoodAngleRad, double timeOfFlightSec, double headingToleranceRad) {

  public BallisticShotParams {
    if (!Double.isFinite(flywheelRps) || flywheelRps < 0.0) {
      throw new IllegalArgumentException("flywheelRps must be finite and >= 0");
    }
    if (!Double.isFinite(hoodAngleRad)) {
      throw new IllegalArgumentException("hoodAngleRad must be finite");
    }
    if (!Double.isFinite(timeOfFlightSec) || timeOfFlightSec < 0.0) {
      throw new IllegalArgumentException("timeOfFlightSec must be finite and >= 0");
    }
    if (!Double.isFinite(headingToleranceRad) || headingToleranceRad <= 0.0) {
      throw new IllegalArgumentException("headingToleranceRad must be finite and > 0");
    }
  }

  /**
   * Linearly interpolates between {@code this} (at {@code t = 0}) and {@code endValue} ({@code t =
   * 1}).
   */
  public BallisticShotParams interpolate(BallisticShotParams endValue, double t) {
    double a = Math.max(0.0, Math.min(1.0, t));
    return new BallisticShotParams(
        flywheelRps + a * (endValue.flywheelRps - flywheelRps),
        hoodAngleRad + a * (endValue.hoodAngleRad - hoodAngleRad),
        timeOfFlightSec + a * (endValue.timeOfFlightSec - timeOfFlightSec),
        headingToleranceRad + a * (endValue.headingToleranceRad - headingToleranceRad));
  }
}
