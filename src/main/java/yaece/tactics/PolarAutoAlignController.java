package yaece.tactics;

import java.util.Objects;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Polar-coordinate close-in auto-alignment controller (distilled from FRC 6328's 2025/2026 {@code
 * DriveToPose}, FRC 254's {@code AutoAlignToPoseCommand}, and FRC 4146's convergence timer).
 *
 * <p>Drives scalar radial distance toward zero along the straight robot-to-goal vector with a
 * feedforward velocity taper window ({@code ffMinRadius..ffMaxRadius}), dynamic CG/mechanism
 * extension derating, optional heading {@code omegaOverride} (so SOTM aiming can own rotation while
 * this controller translates), a continuous convergence hold timer, and a safety timeout.
 */
public final class PolarAutoAlignController {
  public record Config(
      double driveKp,
      double thetaKp,
      double thetaKd,
      double maxLinearSpeedMps,
      double minDeratedLinearSpeedMps,
      double maxLinearAccelMps2,
      double maxAngularSpeedRadPerSec,
      double minDeratedAngularSpeedRadPerSec,
      double ffMinRadiusMeters,
      double ffMaxRadiusMeters,
      double positionToleranceMeters,
      double headingToleranceDeg,
      double convergenceHoldSec,
      double timeoutSec) {
    public static final Config FRC_DEFAULT =
        new Config(
            1.8,
            5.0,
            0.5,
            4.0,
            1.5,
            4.0,
            Math.toRadians(500.0),
            Math.toRadians(120.0),
            0.01,
            0.05,
            0.02,
            1.5,
            0.12,
            5.0);
  }

  public record ChassisVelocityCommand(
      double vxFieldMps,
      double vyFieldMps,
      double omegaRadPerSec,
      boolean atGoal,
      boolean timedOut,
      double distanceErrorMeters,
      double headingErrorDeg) {}

  private final Config config;
  private double startTimeSec = -1.0;
  private double lastTimeSec = -1.0;
  private double lastLinearSpeedCmd = 0.0;
  private double lastHeadingErrRad = 0.0;
  private double convergedDurationSec = 0.0;

  public PolarAutoAlignController() {
    this(Config.FRC_DEFAULT);
  }

  public PolarAutoAlignController(Config config) {
    this.config = Objects.requireNonNull(config, "config must not be null");
  }

  public void reset() {
    startTimeSec = -1.0;
    lastTimeSec = -1.0;
    lastLinearSpeedCmd = 0.0;
    lastHeadingErrRad = 0.0;
    convergedDurationSec = 0.0;
  }

  /**
   * Computes field-relative chassis velocities {@code (vx, vy, omega)} to align {@code currentPose}
   * to {@code targetPose}.
   *
   * @param currentPose live robot Pose2d
   * @param targetPose desired goal Pose2d
   * @param timestampSec monotonic FPGA timestamp in seconds
   * @param mechanismExtensionPercent normalized {@code [0..1]} elevator/arm extension for CG
   *     derating
   * @param omegaOverrideRadPerSec optional external heading rate (e.g., from SOTM aimer)
   */
  public ChassisVelocityCommand calculate(
      Pose2d currentPose,
      Pose2d targetPose,
      double timestampSec,
      double mechanismExtensionPercent,
      OptionalDouble omegaOverrideRadPerSec) {
    Objects.requireNonNull(currentPose, "currentPose must not be null");
    Objects.requireNonNull(targetPose, "targetPose must not be null");
    Objects.requireNonNull(omegaOverrideRadPerSec, "omegaOverrideRadPerSec must not be null");

    if (startTimeSec < 0.0) {
      startTimeSec = timestampSec;
    }
    double dt =
        lastTimeSec < 0.0 ? 0.02 : Math.max(1e-3, Math.min(0.25, timestampSec - lastTimeSec));
    lastTimeSec = timestampSec;

    double elapsed = timestampSec - startTimeSec;
    boolean timedOut = config.timeoutSec() > 0.0 && elapsed >= config.timeoutSec();

    Translation2d delta = targetPose.getTranslation().minus(currentPose.getTranslation());
    double dist = delta.getNorm();
    double headingErrRad =
        wrapRad(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
    double headingErrDeg = Math.toDegrees(Math.abs(headingErrRad));

    // Convergence hold timer (rejects passing through setpoint without settling)
    if (dist <= config.positionToleranceMeters() && headingErrDeg <= config.headingToleranceDeg()) {
      convergedDurationSec += dt;
    } else {
      convergedDurationSec = 0.0;
    }
    boolean atGoal = convergedDurationSec >= config.convergenceHoldSec();

    if (timedOut || atGoal) {
      lastLinearSpeedCmd = 0.0;
      lastHeadingErrRad = headingErrRad;
      return new ChassisVelocityCommand(0.0, 0.0, 0.0, atGoal, timedOut, dist, headingErrDeg);
    }

    // Dynamic CG derating above 35% extension (FRC 6328 2025 pattern)
    double extNorm = Math.max(0.0, Math.min(1.0, (mechanismExtensionPercent - 0.35) / 0.65));
    double extScale = Math.sqrt(extNorm);
    double maxSpeed = lerp(config.maxLinearSpeedMps(), config.minDeratedLinearSpeedMps(), extScale);
    double maxOmega =
        lerp(config.maxAngularSpeedRadPerSec(), config.minDeratedAngularSpeedRadPerSec(), extScale);

    // Polar radial speed with trapezoidal braking curve + FF taper window
    double brakingSpeed = Math.sqrt(2.0 * config.maxLinearAccelMps2() * dist);
    double profileSpeed = Math.min(maxSpeed, brakingSpeed);
    double ffSpan = Math.max(1e-4, config.ffMaxRadiusMeters() - config.ffMinRadiusMeters());
    double ffScaler = Math.max(0.0, Math.min(1.0, (dist - config.ffMinRadiusMeters()) / ffSpan));

    double desiredSpeed =
        Math.min(maxSpeed, config.driveKp() * dist + profileSpeed * ffScaler * 0.5);
    double maxStep = config.maxLinearAccelMps2() * dt;
    double cmdSpeed =
        Math.max(
            0.0,
            Math.min(
                maxSpeed,
                lastLinearSpeedCmd
                    + Math.max(-maxStep, Math.min(maxStep, desiredSpeed - lastLinearSpeedCmd))));
    lastLinearSpeedCmd = cmdSpeed;

    double vx = dist > 1e-5 ? (delta.getX() / dist) * cmdSpeed : 0.0;
    double vy = dist > 1e-5 ? (delta.getY() / dist) * cmdSpeed : 0.0;

    double omega;
    if (omegaOverrideRadPerSec.isPresent()) {
      omega = Math.max(-maxOmega, Math.min(maxOmega, omegaOverrideRadPerSec.getAsDouble()));
    } else {
      double dErr = wrapRad(headingErrRad - lastHeadingErrRad) / dt;
      double rawOmega = config.thetaKp() * headingErrRad + config.thetaKd() * dErr;
      omega = Math.max(-maxOmega, Math.min(maxOmega, rawOmega));
    }
    lastHeadingErrRad = headingErrRad;

    return new ChassisVelocityCommand(vx, vy, omega, false, false, dist, headingErrDeg);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static double wrapRad(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
