package yaece.season;

import java.util.Objects;
import java.util.Set;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * Immutable definition of a field scoring target (Blue-origin canonical frame) with separate Auto
 * and Teleop point values and objective tags.
 */
public record ScoringSiteSpec(
    String id,
    Pose3d bluePose3d,
    Transform3d approachOffset,
    Set<String> acceptedPieceIds,
    double autoPoints,
    double teleopPoints,
    int siteCapacity,
    Pose2d stagingHoldPose2d,
    Set<String> tags,
    String dynamicAvailabilityKey) {

  public ScoringSiteSpec {
    if (id == null || id.isBlank()) {
      throw new IllegalArgumentException("ScoringSiteSpec.id must be non-blank");
    }
    Objects.requireNonNull(bluePose3d, "ScoringSiteSpec.bluePose3d must not be null");
    Objects.requireNonNull(approachOffset, "ScoringSiteSpec.approachOffset must not be null");
    Objects.requireNonNull(acceptedPieceIds, "ScoringSiteSpec.acceptedPieceIds must not be null");
    acceptedPieceIds = Set.copyOf(acceptedPieceIds);
    if (acceptedPieceIds.isEmpty()) {
      throw new IllegalArgumentException("ScoringSiteSpec.acceptedPieceIds must not be empty");
    }
    if (!Double.isFinite(autoPoints) || autoPoints < 0.0) {
      throw new IllegalArgumentException("ScoringSiteSpec.autoPoints must be finite and >= 0");
    }
    if (!Double.isFinite(teleopPoints) || teleopPoints < 0.0) {
      throw new IllegalArgumentException("ScoringSiteSpec.teleopPoints must be finite and >= 0");
    }
    if (siteCapacity == 0 || siteCapacity < -1) {
      throw new IllegalArgumentException(
          "ScoringSiteSpec.siteCapacity must be >= 1 or -1 (unlimited)");
    }
    Objects.requireNonNull(stagingHoldPose2d, "ScoringSiteSpec.stagingHoldPose2d must not be null");
    tags = tags == null ? Set.of() : Set.copyOf(tags);
    dynamicAvailabilityKey = dynamicAvailabilityKey == null ? "" : dynamicAvailabilityKey;
  }

  /** Returns the point value for scoring at this site during the specified match period. */
  public double pointsFor(MatchPeriod period) {
    Objects.requireNonNull(period, "period must not be null");
    return period == MatchPeriod.AUTO ? autoPoints : teleopPoints;
  }

  /** Returns whether this site accepts the given game-piece ID. */
  public boolean acceptsPiece(String pieceId) {
    return pieceId != null && acceptedPieceIds.contains(pieceId);
  }

  /** Returns whether this site carries the given tag. */
  public boolean hasTag(String tag) {
    return tag != null && tags.contains(tag);
  }

  /** Returns whether this site has unlimited scoring capacity. */
  public boolean isUnlimitedCapacity() {
    return siteCapacity == -1;
  }

  public Pose3d poseFor(
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    return FieldMirroring.mirror(bluePose3d, alliance, fieldLengthMeters, fieldWidthMeters, mode);
  }

  public Pose3d approachPoseFor(
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    return FieldMirroring.mirror(
        bluePose3d.plus(approachOffset), alliance, fieldLengthMeters, fieldWidthMeters, mode);
  }

  public Pose2d stagingPoseFor(
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    return FieldMirroring.mirror(
        stagingHoldPose2d, alliance, fieldLengthMeters, fieldWidthMeters, mode);
  }
}
