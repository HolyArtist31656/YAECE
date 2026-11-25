// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.perception;

import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;

/**
 * Sensor-agnostic 6-DoF (or monocular-fallback) game-piece observation fed into YAECE from any
 * vision, RGB-D, or simulation source.
 *
 * @param visionClassId integer class ID matching a {@link
 *     yaece.season.GamePieceSpec#visionClassId()}
 * @param pose3d observed 3D position and orientation in {@code frame}
 * @param frame {@link ObservationFrame#FIELD} or {@link ObservationFrame#ROBOT}
 * @param positionConfidence confidence in [0, 1] for the 3D translation
 * @param orientationConfidence confidence in [0, 1] for the 3D orientation (0 if monocular prior)
 * @param hasFull6DoFOrientation true when roll/pitch/yaw were measured (RGB-D/OBB/keypoints)
 * @param timestampSec capture timestamp in seconds (same time base as ego pose updates)
 */
public record GamePieceObservation6DoF(
    int visionClassId,
    Pose3d pose3d,
    ObservationFrame frame,
    double positionConfidence,
    double orientationConfidence,
    boolean hasFull6DoFOrientation,
    double timestampSec) {

  public GamePieceObservation6DoF {
    if (visionClassId < 0) {
      throw new IllegalArgumentException("visionClassId must be >= 0");
    }
    Objects.requireNonNull(pose3d, "pose3d must not be null");
    Objects.requireNonNull(frame, "frame must not be null");
    positionConfidence = clamp01(positionConfidence);
    orientationConfidence = clamp01(orientationConfidence);
    if (!Double.isFinite(timestampSec) || timestampSec < 0.0) {
      throw new IllegalArgumentException("timestampSec must be finite and >= 0");
    }
  }

  /** Convenience factory for a full 6-DoF field-frame observation. */
  public static GamePieceObservation6DoF field6DoF(
      int visionClassId, Pose3d fieldPose, double confidence, double timestampSec) {
    return new GamePieceObservation6DoF(
        visionClassId,
        fieldPose,
        ObservationFrame.FIELD,
        confidence,
        confidence,
        true,
        timestampSec);
  }

  /** Convenience factory for a full 6-DoF robot-frame observation. */
  public static GamePieceObservation6DoF robot6DoF(
      int visionClassId, Pose3d robotRelativePose, double confidence, double timestampSec) {
    return new GamePieceObservation6DoF(
        visionClassId,
        robotRelativePose,
        ObservationFrame.ROBOT,
        confidence,
        confidence,
        true,
        timestampSec);
  }

  private static double clamp01(double v) {
    if (!Double.isFinite(v) || v < 0.0) {
      return 0.0;
    }
    return Math.min(1.0, v);
  }
}
