// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.bridges;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.CameraBumperBearing;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.season.SeasonConfig;

class YallTargetBridgeTest {

  @Test
  void raycastsFloorGamePieceAndComputesRobotFrameBumperBearingUsingYallSignConvention() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    // Camera mounted at z=1.06m, pitched down 26.565 deg (~atan(0.5)) and yawed +10 deg left
    Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.0, 0.0, 1.06),
            new Rotation3d(0.0, Math.atan(0.5), Math.toRadians(10.0)));
    YallTargetBridge bridge = new YallTargetBridge(season, robotToCam);

    Pose3d ego = new Pose3d(new Translation3d(2.0, 4.0, 0.0), new Rotation3d());
    // Target +10 deg RIGHT in YALL convention cancels the +10 deg LEFT camera yaw -> straight +X
    Optional<GamePieceObservation6DoF> obs = bridge.toFloorGamePiece(0, 10.0, 0.0, 0.9, ego, 15.0);
    assertTrue(obs.isPresent());
    assertEquals(4.0, obs.get().pose3d().getX(), 0.05);
    assertEquals(4.0, obs.get().pose3d().getY(), 0.05);
    assertEquals(0.06, obs.get().pose3d().getZ(), 1e-6);

    // Target -15 deg right (= +15 deg left) + 10 deg left camera yaw = +25 deg CCW robot bearing
    CameraBumperBearing bearing =
        bridge.toRobotBumperBearing(-15.0, RobotAffiliation.OPPONENT, 0.92, 3.0, 15.0);
    assertEquals(Math.toRadians(25.0), bearing.bearingRad(), 1e-6);
    assertEquals(RobotAffiliation.OPPONENT, bearing.affiliation());
  }

  @Test
  void rejectsNonFiniteInputs() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    YallTargetBridge bridge = new YallTargetBridge(season, new Transform3d());
    Pose3d ego = new Pose3d();

    assertThrows(
        IllegalArgumentException.class,
        () -> bridge.toFloorGamePiece(0, Double.NaN, 0.0, 0.9, ego, 1.0));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            bridge.toRobotBumperBearing(
                Double.POSITIVE_INFINITY, RobotAffiliation.ALLY, 0.8, 2.0, 1.0));
  }

  @Test
  void seededPropertyFuzzBumperBearingsStayFiniteAndSymmetric() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    YallTargetBridge bridge = new YallTargetBridge(season, new Transform3d());
    SplittableRandom rng = new SplittableRandom(20271125L);

    for (int i = 0; i < 500; i++) {
      double yawRightDeg = rng.nextDouble(-60.0, 60.0);
      CameraBumperBearing right =
          bridge.toRobotBumperBearing(yawRightDeg, RobotAffiliation.OPPONENT, 0.85, 2.5, 1.0);
      CameraBumperBearing left =
          bridge.toRobotBumperBearing(-yawRightDeg, RobotAffiliation.OPPONENT, 0.85, 2.5, 1.0);
      assertTrue(Double.isFinite(right.bearingRad()));
      assertEquals(-right.bearingRad(), left.bearingRad(), 1e-12);
    }
  }
}
