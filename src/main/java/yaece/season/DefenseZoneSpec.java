package yaece.season;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/** Named defensive patrol/blocking corridor (canonical Blue-origin coordinates). */
public record DefenseZoneSpec(
    String id, List<Translation2d> blueVertices, Pose2d blueAnchorPose2d, double priorityWeight) {

  public DefenseZoneSpec {
    if (id == null || id.isBlank()) {
      throw new IllegalArgumentException("DefenseZoneSpec.id must be non-blank");
    }
    Objects.requireNonNull(blueVertices, "DefenseZoneSpec.blueVertices must not be null");
    blueVertices = List.copyOf(blueVertices);
    if (blueVertices.size() < 3) {
      throw new IllegalArgumentException(
          "DefenseZoneSpec.blueVertices must contain at least 3 vertices");
    }
    Objects.requireNonNull(blueAnchorPose2d, "DefenseZoneSpec.blueAnchorPose2d must not be null");
    if (!Double.isFinite(priorityWeight) || priorityWeight <= 0.0) {
      throw new IllegalArgumentException("DefenseZoneSpec.priorityWeight must be finite and > 0");
    }
  }

  public Pose2d anchorPoseFor(
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    return FieldMirroring.mirror(
        blueAnchorPose2d, alliance, fieldLengthMeters, fieldWidthMeters, mode);
  }
}
