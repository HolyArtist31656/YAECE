// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.ledger.ScoreProjection;
import yaece.perception.RobotAffiliation;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;
import yaece.world.OpponentPinState;
import yaece.world.PinStatus;
import yaece.world.TrackedRobot;
import yaece.world.WorldSnapshot;

class DefenseInterceptorTest {

  @Test
  void clampsBlockingPoseOutsideActiveEndgameNoContactZone() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    DefenseInterceptor def = new DefenseInterceptor(season, RobotKinematicsSpec.FRC_SWERVE_DEFAULT);

    // Opponent moving right into opponent_tower_endgame zone [14.20..16.54, 2.50..5.57] during last
    // 20s
    TrackedRobot opp =
        new TrackedRobot(
            7,
            RobotAffiliation.OPPONENT,
            new Pose3d(new Translation3d(13.5, 4.0, 0.0), new Rotation3d()),
            2.5,
            0.0,
            0.45,
            0.1,
            0.08,
            1.0,
            false,
            130.0);
    WorldSnapshot world =
        new WorldSnapshot(
            130.0,
            new Pose3d(new Translation3d(11.0, 4.0, 0.0), new Rotation3d()),
            0.0,
            0.0,
            List.of(),
            List.of(opp),
            Map.of());

    TacticalRecommendation rec =
        def.evaluate(
            world,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            20.0,
            DefenseStrategyMode.HIGHEST_THREAT,
            null);

    assertEquals(EncounterIntent.DEFEND_INTERCEPT, rec.intent());
    assertTrue(rec.stowMechanisms());
    Translation2d targetXy = rec.targetPose3d().getTranslation().toTranslation2d();
    // Must stay outside X=14.20 minus bumper(0.45) + safetyBuffer(0.45) = 13.30
    assertTrue(
        targetXy.getX() <= 13.35,
        "Expected intercept clamped outside Endgame zone, got x=" + targetXy.getX());
  }

  @Test
  void triggersPinRetreatWhenBackoffRequiredAndAdaptsToZoneDefenseWhenLeading() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    DefenseInterceptor def = new DefenseInterceptor(season, RobotKinematicsSpec.FRC_SWERVE_DEFAULT);

    TrackedRobot opp =
        new TrackedRobot(
            3,
            RobotAffiliation.OPPONENT,
            new Pose3d(new Translation3d(10.0, 6.0, 0.0), new Rotation3d()),
            0.0,
            0.0,
            0.45,
            0.95,
            0.05,
            1.0,
            false,
            50.0);

    // 1. Pin backoff triggered -> must emit DEFEND_PIN_RETREAT
    OpponentPinState activePin =
        new OpponentPinState(3, PinStatus.PINNING_ACTIVE, 2.3, 0.0, true, 0.1);
    WorldSnapshot pinnedWorld =
        new WorldSnapshot(
            50.0,
            new Pose3d(new Translation3d(9.0, 6.0, 0.0), new Rotation3d()),
            0.0,
            0.0,
            List.of(),
            List.of(opp),
            Map.of(3, activePin));

    TacticalRecommendation retreat =
        def.evaluate(
            pinnedWorld,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            100.0,
            DefenseStrategyMode.AUTO_ADAPTIVE,
            null);
    assertEquals(EncounterIntent.DEFEND_PIN_RETREAT, retreat.intent());
    assertTrue(
        retreat
                .targetPose3d()
                .getTranslation()
                .toTranslation2d()
                .getDistance(new Translation2d(10.0, 6.0))
            >= 2.5);

    // 2. No pin + comfortable projected lead (+30 pts) -> AUTO_ADAPTIVE chooses ZONE_DEFENSE
    WorldSnapshot clearWorld =
        new WorldSnapshot(
            55.0,
            new Pose3d(new Translation3d(9.0, 6.0, 0.0), new Rotation3d()),
            0.0,
            0.0,
            List.of(),
            List.of(opp),
            Map.of());
    ScoreProjection leading = new ScoreProjection(55.0, 90.0, 60.0, 1.5, 1.0, 180.0, 150.0, 0.9);
    TacticalRecommendation zoneRec =
        def.evaluate(
            clearWorld,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            95.0,
            DefenseStrategyMode.AUTO_ADAPTIVE,
            leading);
    assertEquals(DefenseStrategyMode.ZONE_DEFENSE, zoneRec.activeDefenseStrategy());
  }
}
