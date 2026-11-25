// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.perception;

import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Tilt-compensated monocular ground-plane raycaster for teams without RGB-D or 6-DoF coprocessors.
 *
 * <p>Uses the live 3D ego pose (including roll/pitch from QuestNav or IMU) and the camera's {@code
 * robotToCamera} transform to intersect the camera ray with {@code z = restHeightMeters}.
 */
public final class MonocularRaycaster {
  private static final double MIN_DOWNWARD_Z = -1e-4;
  private static final double MAX_RAY_DISTANCE_METERS = 7.5;

  private MonocularRaycaster() {}

  /**
   * Projects a 2D camera target ray ({@code targetYawRad} left-positive, {@code targetPitchRad}
   * up-positive) onto the horizontal plane {@code z = restHeightMeters} in field coordinates.
   */
  public static Optional<GamePieceObservation6DoF> raycastToFloor(
      int visionClassId,
      double targetYawRad,
      double targetPitchRad,
      Transform3d robotToCamera,
      Pose3d egoPose3d,
      double restHeightMeters,
      double detectionConfidence,
      double timestampSec) {
    Pose3d camFieldPose = egoPose3d.plus(robotToCamera);

    // Unit ray in camera frame (+X forward, +Y left, +Z up)
    double cosPitch = Math.cos(targetPitchRad);
    Translation3d rayCam =
        new Translation3d(
            cosPitch * Math.cos(targetYawRad),
            cosPitch * Math.sin(targetYawRad),
            Math.sin(targetPitchRad));

    Translation3d rayField = rayCam.rotateBy(camFieldPose.getRotation());
    if (rayField.getZ() >= MIN_DOWNWARD_Z) {
      return Optional.empty();
    }

    double t = (restHeightMeters - camFieldPose.getZ()) / rayField.getZ();
    if (t <= 0.0 || t > MAX_RAY_DISTANCE_METERS) {
      return Optional.empty();
    }

    Translation3d hit =
        new Translation3d(
            camFieldPose.getX() + t * rayField.getX(),
            camFieldPose.getY() + t * rayField.getY(),
            restHeightMeters);

    // Distance-attenuated confidence (monocular floor raycast degrades past ~3.5 m)
    double distAtten = Math.max(0.1, 1.0 - (t / MAX_RAY_DISTANCE_METERS) * 0.65);
    return Optional.of(
        new GamePieceObservation6DoF(
            visionClassId,
            new Pose3d(hit, new Rotation3d()),
            ObservationFrame.FIELD,
            detectionConfidence * distAtten,
            0.0,
            false,
            timestampSec));
  }
}
