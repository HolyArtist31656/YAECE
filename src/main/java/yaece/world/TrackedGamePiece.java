package yaece.world;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;

/** Immutable snapshot of a tracked 6-DoF game piece on the field. */
public record TrackedGamePiece(
    int trackId,
    String pieceId,
    int visionClassId,
    Pose3d pose3d,
    Translation3d velocityMps,
    double positionUncertaintyMeters,
    double orientationConfidence,
    double existenceConfidence,
    double lastSeenTimestampSec) {

  /** Projects this game piece's 3D translation forward by {@code dtSec} seconds. */
  public Translation3d predictPositionAt(double dtSec) {
    double dt = Math.max(0.0, dtSec);
    return new Translation3d(
        pose3d.getX() + velocityMps.getX() * dt,
        pose3d.getY() + velocityMps.getY() * dt,
        pose3d.getZ() + velocityMps.getZ() * dt);
  }
}
