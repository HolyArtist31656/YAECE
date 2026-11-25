// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import yaece.bridges.YamsSwerveHooks;
import yaece.commands3.YaeceCommandFactory;
import yaece.commands3.YaeceExecutionHooks;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.season.AllianceSide;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;
import yaece.world.FieldWorldModel;

class Phase6ReliabilityAndPathfindingTest {

  @Test
  void fieldPathfinderRoutesAroundRebuiltHubObstacleWhenBlocked() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    FieldPathfinder pathfinder = new FieldPathfinder(rebuilt, 0.43);

    // Straight line from (2.2, 4.03) to (7.0, 4.03) passes through blue_hub_base [4.03..5.15,
    // 3.44..4.63]
    Translation2d start = new Translation2d(2.2, 4.03);
    Translation2d goal = new Translation2d(7.0, 4.03);
    FieldPathfinder.PathResult detour =
        pathfinder.findPath(start, goal, AllianceSide.BLUE, MatchPeriod.TELEOP, 100.0);

    assertTrue(detour.detourRequired(), "Path across REBUILT hub must require a detour");
    assertTrue(
        detour.totalDistanceMeters() > start.getDistance(goal),
        "Detour path length must exceed straight-line Euclidean distance");
    assertTrue(detour.waypoints().size() >= 2, "Detour must include intermediate waypoint(s)");
    assertNotEquals(goal, detour.nextWaypoint(), "Next waypoint should route around hub corner");

    Translation2d openA = new Translation2d(7.0, 2.0);
    Translation2d openB = new Translation2d(8.5, 2.5);
    FieldPathfinder.PathResult direct =
        pathfinder.findPath(openA, openB, AllianceSide.BLUE, MatchPeriod.TELEOP, 100.0);
    assertFalse(direct.detourRequired());
    assertEquals(openA.getDistance(openB), direct.totalDistanceMeters(), 1e-6);
  }

  @Test
  void velocityProjectedArrivalRaceIgnoresOpponentDrivingAway() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine engine =
        YaeceEngine.builder(rebuilt)
            .withAlliance(AllianceSide.BLUE)
            .withKinematics(RobotKinematicsSpec.FRC_SWERVE_DEFAULT)
            .build();

    // Ego at (4.0, 2.0) -> 3.0 m from piece at (7.0, 2.0)
    engine.updateEgoPose(new Pose3d(4.0, 2.0, 0.0, new Rotation3d()), 1.0);
    engine.feedGamePieces(
        List.of(
            GamePieceObservation6DoF.field6DoF(
                0, new Pose3d(7.0, 2.0, 0.075, new Rotation3d()), 0.95, 1.0)));

    // Opponent is closer (at x=8.2, 1.2 m away) BUT sprinting AWAY (+X at 3.5 m/s)
    engine.feedRobots(
        List.of(
            RobotObservation.fieldRobot(
                new Pose3d(8.2, 2.0, 0.0, new Rotation3d()),
                RobotAffiliation.OPPONENT,
                0.95,
                1.0)));
    engine.updateAndEvaluate(1.0, RobotInventoryState.empty(), MatchPeriod.TELEOP, 120.0);

    engine.updateEgoPose(new Pose3d(4.0, 2.0, 0.0, new Rotation3d()), 1.05);
    engine.feedRobots(
        List.of(
            RobotObservation.fieldRobot(
                new Pose3d(8.375, 2.0, 0.0, new Rotation3d()),
                RobotAffiliation.OPPONENT,
                0.95,
                1.05)));
    TacticalRecommendation rec =
        engine.updateAndEvaluate(1.05, RobotInventoryState.empty(), MatchPeriod.TELEOP, 119.95);

    assertEquals(
        EncounterIntent.GATHER,
        rec.intent(),
        "Should still GATHER when closer opponent is moving away from the piece");
  }

  @Test
  void ledgerRecordSelfScoredHonorsDynamicScoringMultiplier() {
    String jsonWith2x =
        """
        {
          "schemaVersion": 1,
          "seasonName": "TEST_2X",
          "fieldLengthMeters": 16.54,
          "fieldWidthMeters": 8.07,
          "mirrorMode": "ROTATIONAL_180",
          "maxTotalHeldGamePieces": 2,
          "matchDurationSeconds": 150.0,
          "gamePieces": [
            {
              "id": "fuel",
              "visionClassId": 0,
              "dimensionsMeters": [0.15, 0.15, 0.15],
              "restHeightMeters": 0.075,
              "symmetry": "SPHERICAL",
              "maxHeldSimultaneously": 2,
              "requireOrientationForIntake": false
            }
          ],
          "scoringSites": [
            {
              "id": "hub",
              "pose3d": [2.0, 4.0, 1.8, 0.0, 0.0, 0.0],
              "approachOffsetMeters": [0.0, 0.0, -1.8, 0.0, 0.0, 0.0],
              "acceptedPieces": ["fuel"],
              "autoPoints": 1.0,
              "teleopPoints": 2.0,
              "stagingHoldPose2d": [1.5, 4.0, 0.0],
              "tags": ["hub_fuel"],
              "dynamicAvailabilityKey": "hub_active"
            }
          ],
          "scoringIntervals": [
            {
              "id": "endgame_2x",
              "startElapsedSec": 120.0,
              "endElapsedSec": 150.0,
              "matchingSiteTags": ["hub_fuel"],
              "active": true,
              "pointMultiplier": 2.0,
              "flatBonusPoints": 1.0
            }
          ]
        }
        """;
    SeasonConfig season = SeasonConfig.fromJsonString(jsonWith2x);
    YaeceEngine engine = YaeceEngine.builder(season).build();
    engine.setDynamicKeyActive("hub_active", true);

    // At t = 130s (inside 2x + 1 flat bonus interval): 2.0 * 2.0 + 1.0 = 5.0 pts
    engine.recordSelfScored("hub", "fuel", MatchPeriod.TELEOP, 130.0);
    assertEquals(5.0, engine.getLedger().getSelfScoredPoints(), 1e-6);
  }

  @Test
  void fieldWorldModelSupportsConcurrentIngestionAcrossThreads() throws InterruptedException {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    FieldWorldModel world = new FieldWorldModel(rebuilt, 0.43);

    AtomicBoolean failed = new AtomicBoolean(false);
    Thread visionThread =
        new Thread(
            () -> {
              try {
                for (int i = 0; i < 200; i++) {
                  double t = i * 0.01;
                  world.addGamePieceObservations(
                      List.of(
                          GamePieceObservation6DoF.field6DoF(
                              0, new Pose3d(6.0, 4.0, 0.075, new Rotation3d()), 0.9, t)));
                  world.addRobotObservations(
                      List.of(
                          RobotObservation.fieldRobot(
                              new Pose3d(10.0, 4.0, 0.0, new Rotation3d()),
                              RobotAffiliation.OPPONENT,
                              0.9,
                              t)));
                }
              } catch (Throwable ex) {
                failed.set(true);
              }
            });

    visionThread.start();
    for (int i = 0; i < 100; i++) {
      world.updateEgoPose(new Pose3d(2.5, 4.0, 0.0, new Rotation3d()), i * 0.02);
      world.step(i * 0.02);
    }
    visionThread.join(2000);
    assertFalse(failed.get(), "Concurrent vision ingestion must not throw exceptions");
  }

  @Test
  void continuousCommandFactoryAndDiagnosticReportValidateCleanly() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine engine = YaeceEngine.builder(rebuilt).build();
    YaeceExecutionHooks hooks =
        YamsSwerveHooks.create(
            p -> Command.noRequirements(co -> {}).named("Drive"), null, null, null);

    Command continuous =
        YamsSwerveHooks.createContinuous(
            engine,
            supplier -> Command.noRequirements(co -> {}).named("ContinuousDrive"),
            null,
            () -> false);
    assertEquals("YAECE Continuous Encounter", continuous.name());

    Command auto = YaeceCommandFactory.runAutonomousEncounter(engine, hooks, () -> false);
    assertEquals("YAECE Autonomous Encounter", auto.name());

    YaeceDiagnostic.DiagnosticReport report =
        YaeceDiagnostic.generateReport(rebuilt, engine, hooks);
    assertTrue(report.healthy(), "Bundled 2026 REBUILT season must pass YaeceDiagnostic");
    assertTrue(report.formattedReport().contains("hub_active"));

    var obs = rebuilt.staticObstacles().get(0);
    assertSame(
        rebuilt.resolvedObstaclePolygon(obs, AllianceSide.BLUE),
        rebuilt.resolvedObstaclePolygon(obs, AllianceSide.BLUE));
  }
}
