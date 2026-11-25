package yaece.world;

/**
 * Unified 2027 vision gating & dynamic covariance policy combining:
 *
 * <ul>
 *   <li>Pre-enable Phase 1 hard field alignment / MegaTag1 gyro zeroing ({@code >= 2} tags, {@code
 *       >= 60%} RANSAC inliers).
 *   <li>Enabled Phase 2 VIO/odometry-primary selective updates ({@code <= 360 deg/s}, {@code <= 2.0
 *       m} jump, {@code >= 80%} inliers, {@code baseStdDev * d^2 / tagCount} dynamic scaling, high
 *       yaw std-dev so gyro/VIO owns heading).
 *   <li>3+ tag shooter-camera trust boost ({@code 0.35x} XY std-dev multiplier).
 * </ul>
 */
public final class VisionFusionPolicy {
  public static final double HIGH_YAW_STD_DEV_RAD = 9999999.0;

  public record FusionDecision(
      boolean accepted,
      boolean hardResetPose,
      double xyStdDevMeters,
      double thetaStdDevRad,
      String reason) {}

  private final double baseStdDevMeters;
  private final double maxAngularVelDegPerSec;
  private final double maxPoseJumpMeters;
  private final double shooterCameraBoost;

  public VisionFusionPolicy() {
    this(0.06, 360.0, 2.0, 0.35);
  }

  public VisionFusionPolicy(
      double baseStdDevMeters,
      double maxAngularVelDegPerSec,
      double maxPoseJumpMeters,
      double shooterCameraBoost) {
    this.baseStdDevMeters = baseStdDevMeters;
    this.maxAngularVelDegPerSec = maxAngularVelDegPerSec;
    this.maxPoseJumpMeters = maxPoseJumpMeters;
    this.shooterCameraBoost = shooterCameraBoost;
  }

  public FusionDecision evaluate(
      boolean robotEnabled,
      int tagCount,
      double avgTagDistanceMeters,
      double ransacInlierRatio,
      double poseJumpMeters,
      double omegaDegPerSec,
      boolean isShooterCamera) {
    if (tagCount <= 0) {
      return new FusionDecision(
          false, false, Double.POSITIVE_INFINITY, HIGH_YAW_STD_DEV_RAD, "no_tags");
    }
    if (Math.abs(omegaDegPerSec) > maxAngularVelDegPerSec) {
      return new FusionDecision(
          false, false, Double.POSITIVE_INFINITY, HIGH_YAW_STD_DEV_RAD, "high_angular_rate");
    }

    double dist = Math.max(0.5, avgTagDistanceMeters);
    double xyStd = baseStdDevMeters * (dist * dist) / tagCount;

    if (!robotEnabled) {
      // Phase 1 pre-enable: MegaTag1 / initial field alignment + gyro zeroing
      if (tagCount >= 2 && ransacInlierRatio >= 0.60) {
        return new FusionDecision(true, true, xyStd * 0.5, 0.02, "pre_enable_phase1_align");
      }
      return new FusionDecision(
          false,
          false,
          Double.POSITIVE_INFINITY,
          HIGH_YAW_STD_DEV_RAD,
          "pre_enable_low_tags_or_inliers");
    }

    // Phase 2 enabled: VIO/odometry-primary selective translational corrections
    if (poseJumpMeters > maxPoseJumpMeters) {
      return new FusionDecision(
          false, false, Double.POSITIVE_INFINITY, HIGH_YAW_STD_DEV_RAD, "pose_jump_rejected");
    }
    if (ransacInlierRatio < 0.80) {
      return new FusionDecision(
          false, false, Double.POSITIVE_INFINITY, HIGH_YAW_STD_DEV_RAD, "low_ransac_inliers");
    }
    if (tagCount < 2 && dist > 3.5) {
      return new FusionDecision(
          false, false, Double.POSITIVE_INFINITY, HIGH_YAW_STD_DEV_RAD, "far_single_tag_rejected");
    }

    if (isShooterCamera && tagCount >= 3) {
      xyStd *= shooterCameraBoost;
    }
    return new FusionDecision(true, false, xyStd, HIGH_YAW_STD_DEV_RAD, "phase2_accepted");
  }
}
