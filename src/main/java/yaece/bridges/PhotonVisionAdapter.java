package yaece.bridges;

import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import yaece.perception.CameraBumperBearing;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.MonocularRaycaster;
import yaece.perception.RobotAffiliation;
import yaece.season.GamePieceSpec;
import yaece.season.SeasonConfig;

/**
 * Zero-dependency PhotonVision helper converting 2D {@code PhotonTrackedTarget} yaw/pitch angles
 * into tilt-compensated floor {@link GamePieceObservation6DoF} records or robot-frame {@link
 * CameraBumperBearing} detections for LiDAR fusion.
 */
public final class PhotonVisionAdapter {
  private final SeasonConfig season;
  private final Transform3d robotToCamera;

  public PhotonVisionAdapter(SeasonConfig season, Transform3d robotToCamera) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.robotToCamera = Objects.requireNonNull(robotToCamera, "robotToCamera must not be null");
  }

  /**
   * Converts a PhotonVision 2D object detection (yaw/pitch in degrees, left/up positive) into a
   * tilt-compensated field-frame {@link GamePieceObservation6DoF} using the piece's configured
   * {@code restHeightMeters}.
   */
  public Optional<GamePieceObservation6DoF> toFloorGamePiece(
      int detectedClassId,
      double targetYawDeg,
      double targetPitchDeg,
      double confidence,
      Pose3d egoPose3d,
      double captureTimestampSec) {
    Optional<GamePieceSpec> specOpt = season.findGamePieceByVisionClass(detectedClassId);
    if (specOpt.isEmpty()) {
      return Optional.empty();
    }
    return MonocularRaycaster.raycastToFloor(
        detectedClassId,
        Math.toRadians(targetYawDeg),
        Math.toRadians(targetPitchDeg),
        robotToCamera,
        egoPose3d,
        specOpt.get().restHeightMeters(),
        confidence,
        captureTimestampSec);
  }

  /**
   * Converts a PhotonVision bumper-color detection yaw (degrees, left positive) into a robot-frame
   * {@link CameraBumperBearing} adjusted by the camera's mounted yaw offset.
   */
  public CameraBumperBearing toRobotBumperBearing(
      double targetYawDeg,
      RobotAffiliation affiliation,
      double confidence,
      double optionalEstimatedRangeMeters,
      double captureTimestampSec) {
    double robotFrameBearingRad = robotToCamera.getRotation().getZ() + Math.toRadians(targetYawDeg);
    return new CameraBumperBearing(
        robotFrameBearingRad,
        affiliation,
        confidence,
        optionalEstimatedRangeMeters,
        captureTimestampSec);
  }
}
