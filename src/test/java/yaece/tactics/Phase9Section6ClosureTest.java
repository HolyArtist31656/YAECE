// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.ballistics.BallisticShotParams;
import yaece.ballistics.BallisticTable;
import yaece.season.AllianceSide;
import yaece.season.CompositeZone;
import yaece.season.FieldType;
import yaece.season.SeasonConfig;
import yaece.world.RobotHealthState;

class Phase9Section6ClosureTest {

  private static final String HUB_BALLISTICS_JSON =
      """
      {
        "tableName": "rebuilt_hub",
        "entries": [
          {
            "distanceMeters": 1.5,
            "flywheelRps": 50.0,
            "hoodAngleRad": 0.40,
            "timeOfFlightSec": 0.40,
            "headingToleranceDeg": 5.0
          },
          {
            "distanceMeters": 5.5,
            "flywheelRps": 70.0,
            "hoodAngleRad": 0.60,
            "timeOfFlightSec": 1.20,
            "headingToleranceDeg": 2.5
          }
        ]
      }
      """;

  @Test
  void fieldTypeAndCompositeZoneSupportBooleanSetOperations() {
    assertTrue(FieldType.WELDED.nominalLengthMeters() > FieldType.ANDYMARK.nominalLengthMeters());

    CompositeZone outer = CompositeZone.ofRect(0.0, 0.0, 6.0, 6.0);
    CompositeZone hole = CompositeZone.ofRect(2.0, 2.0, 4.0, 4.0);
    CompositeZone ring = outer.subtract(hole);

    assertTrue(ring.contains(new Translation2d(1.0, 1.0)));
    assertFalse(ring.contains(new Translation2d(3.0, 3.0)));
    assertTrue(ring.complement().contains(new Translation2d(3.0, 3.0)));

    CompositeZone sideWing = CompositeZone.ofRect(5.0, 0.0, 8.0, 3.0);
    assertTrue(outer.union(sideWing).contains(new Translation2d(7.0, 1.5)));
    assertTrue(outer.intersect(sideWing).contains(new Translation2d(5.5, 1.5)));
    assertFalse(outer.intersect(sideWing).contains(new Translation2d(7.0, 1.5)));
  }

  @Test
  void ballisticTableInterpolatesFromJsonAndFeedsShootOnTheMoveSolver() {
    BallisticTable hubLut = BallisticTable.fromJson(HUB_BALLISTICS_JSON);
    assertEquals("rebuilt_hub", hubLut.tableName());
    assertEquals(2, hubLut.size());

    // Midpoint at 3.5 m should interpolate linearly between 1.5 m and 5.5 m
    BallisticShotParams mid = hubLut.get(3.5);
    assertEquals(60.0, mid.flywheelRps(), 1e-6);
    assertEquals(0.50, mid.hoodAngleRad(), 1e-6);
    assertEquals(0.80, mid.timeOfFlightSec(), 1e-6);
    assertEquals(Math.toRadians(3.75), mid.headingToleranceRad(), 1e-6);

    // Feeds ShootOnTheMoveSolver directly via tofLookup()
    ShootOnTheMoveSolver solver =
        new ShootOnTheMoveSolver(ShootOnTheMoveSolver.Config.REBUILT_DEFAULT, hubLut.tofLookup());
    var sol =
        solver.solve(
            new org.wpilib.math.geometry.Pose2d(
                2.0, 4.0, new org.wpilib.math.geometry.Rotation2d()),
            0.0,
            1.5,
            0.0,
            new Translation3d(4.625, 4.035, 1.829));
    assertTrue(sol.effectiveTofSec() > 0.3 && sol.effectiveTofSec() < 1.0);
    assertTrue(sol.inRange());
  }

  @Test
  void shotReadinessGateVolleyEnforcesSettleHoldHysteresisTimeoutAndSetShot() {
    ShotReadinessGate gate = new ShotReadinessGate();
    Pose3d target = new Pose3d(2.75, 4.03, 0.0, new Rotation3d());
    Pose3d tight = new Pose3d(2.77, 4.03, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(1.5)));
    Pose3d loose = new Pose3d(2.77, 4.03, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(4.5)));

    // First tick within tight tolerance -> still settling (< 0.04 s)
    var r0 = gate.evaluateVolley(tight, target, 0.05, true, 10.00, 9.99, false, 0.0, false, false);
    assertFalse(r0.readyToScore());

    // After 0.05 s settle hold -> volley starts
    var r1 =
        gate.evaluateVolley(tight, target, 0.05, true, 10.05, 10.04, false, 0.05, false, false);
    assertTrue(r1.readyToScore());

    // Once volley is active, 4.5 deg heading error stays inside 2x keep-feeding hysteresis (6.0
    // deg)
    var r2 =
        gate.evaluateVolley(loose, target, 0.05, true, 10.08, 10.07, false, 0.08, false, false);
    assertTrue(r2.readyToScore(), "Hysteresis should keep feeding up to 2x heading tolerance");

    // Pose-free SetShot fallback releases even when far from target
    gate.reset();
    Pose3d farAway = new Pose3d(8.0, 1.0, 0.0, new Rotation3d());
    var setShot =
        gate.evaluateVolley(farAway, target, 0.0, true, 11.0, -1.0, false, 0.0, false, true);
    assertTrue(setShot.readyToScore());

    // Hard 1.0 s timeout releases
    gate.reset();
    var timedOut =
        gate.evaluateVolley(loose, target, 0.05, true, 12.0, 11.99, false, 1.05, false, false);
    assertTrue(timedOut.readyToScore());
  }

  @Test
  void engineStoresRobotHealthStateAndResetsToNominalOnMatchReset() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine engine = YaeceEngine.builder(rebuilt).withAlliance(AllianceSide.BLUE).build();

    assertEquals(RobotHealthState.NOMINAL, engine.getRobotHealth());
    RobotHealthState degraded = new RobotHealthState(0.55, true, 0.65, true);
    engine.updateRobotHealth(degraded);
    assertEquals(0.55, engine.getRobotHealth().odometryConfidence(), 1e-6);
    assertEquals(0.65, engine.getRobotHealth().maxSpeedThrottleFactor(), 1e-6);
    assertTrue(engine.getRobotHealth().isSkiddingOrTilted());
    assertTrue(engine.getRobotHealth().collisionDetected());

    engine.resetForMatch();
    assertEquals(RobotHealthState.NOMINAL, engine.getRobotHealth());
  }
}
