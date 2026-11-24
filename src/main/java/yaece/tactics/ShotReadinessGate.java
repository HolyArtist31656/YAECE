package yaece.tactics;

import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;

/**
 * Dual-gated shot and scoring readiness evaluator (distilled from FRC 1678's {@code ShotVerifier} /
 * {@code FollowSyncedTagPIDToPose} and FRC 2910's {@code isReadyToScore}).
 *
 * <p>Separates odometry convergence, chassis tilt, chassis speed, vision freshness, and an
 * autonomous fail-safe wait timeout so a dropped camera frame never deadlocks auto while signaling
 * driver rumble in teleop.
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

  public record ReadinessResult(
      boolean readyToScore, boolean driverRumbleVisionStale, String reason) {}

  private final Config config;

  public ShotReadinessGate() {
    this(Config.FRC_DEFAULT);
  }

  public ShotReadinessGate(Config config) {
    this.config = Objects.requireNonNull(config, "config must not be null");
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

  private static double wrapRad(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
