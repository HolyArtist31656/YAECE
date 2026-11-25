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

  public record ReadinessResult(
      boolean readyToScore, boolean driverRumbleVisionStale, String reason) {}

  private final Config config;
  private final VolleyConfig volleyConfig;
  private double settleStartSec = -1.0;
  private boolean volleyActive = false;

  public ShotReadinessGate() {
    this(Config.FRC_DEFAULT, VolleyConfig.FRC_DEFAULT);
  }

  public ShotReadinessGate(Config config) {
    this(config, VolleyConfig.FRC_DEFAULT);
  }

  public ShotReadinessGate(Config config, VolleyConfig volleyConfig) {
    this.config = Objects.requireNonNull(config, "config must not be null");
    this.volleyConfig = Objects.requireNonNull(volleyConfig, "volleyConfig must not be null");
  }

  /** Resets volley hysteresis and settle hold timers. */
  public void reset() {
    settleStartSec = -1.0;
    volleyActive = false;
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

  private static double wrapRad(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
