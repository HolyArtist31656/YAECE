// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

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
 * Zero-dependency bridge converting vendor-neutral {@code YALL} ({@code yall.results.VisionTarget})
 * normalized target angles into tilt-compensated floor {@link GamePieceObservation6DoF} records or
 * robot-frame {@link CameraBumperBearing} detections for LiDAR fusion.
 *
 * <p>Follows YALL's normalized camera sign convention across PhotonVision, Limelight, and Synapse:
 * {@code targetYawRightDeg} is positive to the right of the crosshair and {@code targetPitchUpDeg}
 * is positive above the crosshair. Internally negates yaw into WPILib's CCW-positive (left-positive)
 * frame before raycasting.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * YallTargetBridge bridge = new YallTargetBridge(seasonConfig, robotToCamera);
 * Optional<GamePieceObservation6DoF> piece =
 *     bridge.toFloorGamePiece(
 *         classId, target.yawDegrees(), target.pitchDegrees(),
 *         target.confidence(), egoPose3d, captureTimestampSec);
 * }</pre>
 */
public final class YallTargetBridge {
  /** Season configuration providing vision class ID to rest-height mappings. */
  private final SeasonConfig season;

  /** Mounting transform from robot center to camera optical frame. */
  private final Transform3d robotToCamera;

  /**
   * Creates a vendor-neutral camera target bridge using YALL's normalized yaw/pitch convention.
   *
   * @param season loaded season configuration containing {@link GamePieceSpec} definitions
   * @param robotToCamera transform from robot center to the camera optical frame
   */
  public YallTargetBridge(SeasonConfig season, Transform3d robotToCamera) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.robotToCamera = Objects.requireNonNull(robotToCamera, "robotToCamera must not be null");
  }

  /**
   * Converts a YALL-normalized 2D object detection ({@code targetYawRightDeg} positive right,
   * {@code targetPitchUpDeg} positive above crosshair) into a tilt-compensated field-frame {@link
   * GamePieceObservation6DoF} using the piece's configured {@code restHeightMeters}.
   *
   * @param detectedClassId vision class ID matching {@link SeasonConfig#findGamePieceByVisionClass}
   * @param targetYawRightDeg target horizontal angle in degrees (positive right, YALL convention)
   * @param targetPitchUpDeg target vertical angle in degrees (positive up, YALL convention)
   * @param confidence detection confidence in {@code [0.0, 1.0]}
   * @param egoPose3d robot 6-DoF pose at {@code captureTimestampSec}
   * @param captureTimestampSec FPGA capture timestamp in seconds
   * @return projected floor game-piece observation, or empty if class is unknown or ray misses floor
   */
  public Optional<GamePieceObservation6DoF> toFloorGamePiece(
      int detectedClassId,
      double targetYawRightDeg,
      double targetPitchUpDeg,
      double confidence,
      Pose3d egoPose3d,
      double captureTimestampSec) {
    if (!Double.isFinite(targetYawRightDeg)
        || !Double.isFinite(targetPitchUpDeg)
        || !Double.isFinite(confidence)
        || !Double.isFinite(captureTimestampSec)) {
      throw new IllegalArgumentException(
          "Camera target angles, confidence, and timestamp must be finite.");
    }
    Optional<GamePieceSpec> specOpt = season.findGamePieceByVisionClass(detectedClassId);
    if (specOpt.isEmpty()) {
      return Optional.empty();
    }
    // YALL yaw is positive-right; MonocularRaycaster expects WPILib CCW (positive-left).
    double yawLeftRad = Math.toRadians(-targetYawRightDeg);
    double pitchUpRad = Math.toRadians(targetPitchUpDeg);
    return MonocularRaycaster.raycastToFloor(
        detectedClassId,
        yawLeftRad,
        pitchUpRad,
        robotToCamera,
        egoPose3d,
        specOpt.get().restHeightMeters(),
        confidence,
        captureTimestampSec);
  }

  /**
   * Converts a YALL-normalized bumper-color detection yaw ({@code targetYawRightDeg} positive right)
   * into a robot-frame CCW-positive {@link CameraBumperBearing} adjusted by the camera's mounted
   * yaw offset.
   *
   * @param targetYawRightDeg target horizontal angle in degrees (positive right, YALL convention)
   * @param affiliation classified robot affiliation (ally or opponent)
   * @param confidence detection confidence in {@code [0.0, 1.0]}
   * @param optionalEstimatedRangeMeters optional monocular range estimate in meters ({@code <= 0} if unknown)
   * @param captureTimestampSec FPGA capture timestamp in seconds
   * @return robot-frame bumper bearing for LiDAR-camera fusion
   */
  public CameraBumperBearing toRobotBumperBearing(
      double targetYawRightDeg,
      RobotAffiliation affiliation,
      double confidence,
      double optionalEstimatedRangeMeters,
      double captureTimestampSec) {
    if (!Double.isFinite(targetYawRightDeg)
        || !Double.isFinite(confidence)
        || !Double.isFinite(optionalEstimatedRangeMeters)
        || !Double.isFinite(captureTimestampSec)) {
      throw new IllegalArgumentException("Bumper bearing inputs must be finite.");
    }
    double robotFrameBearingRad =
        robotToCamera.getRotation().getZ() - Math.toRadians(targetYawRightDeg);
    return new CameraBumperBearing(
        robotFrameBearingRad,
        affiliation,
        confidence,
        optionalEstimatedRangeMeters,
        captureTimestampSec);
  }
}
