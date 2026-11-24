package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.bridges.YacSim3dBridge;
import yaece.console.PlayMode;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.replay.RecordedTick;
import yaece.replay.YaeceInputRecorder;
import yaece.replay.YaeceReplayHarness;
import yaece.scouting.OpponentProfile;
import yaece.scouting.ScoutingPriorRegistry;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;
import yaece.world.TrackedRobot;

class Phase8ScoutingAndYacSimBridgeTest {

  private static final String SCOUTING_JSON =
      """
      {
        "teams": [
          {
            "teamNumber": 254,
            "role": "PRIMARY_SCORER",
            "expectedPointsPerMinute": 40.0,
            "preferredSiteThreatMultiplier": 1.25,
            "maxSpeedMps": 5.2,
            "defenseSusceptibility": 1.2,
            "preferredSiteIds": ["alliance_hub_close"]
          },
          {
            "teamNumber": 9999,
            "role": "DEFENDER",
            "expectedPointsPerMinute": 4.0,
            "preferredSiteThreatMultiplier": 0.8,
            "maxSpeedMps": 4.0,
            "defenseSusceptibility": 0.6,
            "preferredSiteIds": []
          },
          {
            "teamNumber": 1678,
            "role": "ENDGAME_SPECIALIST",
            "expectedPointsPerMinute": 20.0,
            "preferredSiteThreatMultiplier": 1.0,
            "maxSpeedMps": 4.8,
            "defenseSusceptibility": 1.0,
            "preferredSiteIds": []
          }
        ]
      }
      """;

  @Test
  void scoutingPriorsShiftDefenseInterceptToPrimaryScorerOverDefender() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    ScoutingPriorRegistry priors = ScoutingPriorRegistry.fromJson(SCOUTING_JSON);
    assertEquals(3, priors.registeredTeamCount());

    YaeceEngine engine =
        YaeceEngine.builder(rebuilt)
            .withAlliance(AllianceSide.BLUE)
            .withKinematics(RobotKinematicsSpec.FRC_SWERVE_DEFAULT)
            .withScoutingPriors(priors)
            .build();
    engine.setPlayMode(PlayMode.DEFENSE);
    engine.setDefenseStrategyMode(DefenseStrategyMode.HIGHEST_THREAT);

    // Seed two symmetric opponents equidistant from the red hub
    Pose3d ego = new Pose3d(8.0, 4.0, 0.0, new Rotation3d());
    Pose3d oppA = new Pose3d(10.5, 3.0, 0.0, new Rotation3d());
    Pose3d oppB = new Pose3d(10.5, 5.0, 0.0, new Rotation3d());

    engine.updateEgoPose(ego, 1.00);
    engine.feedRobots(
        List.of(
            RobotObservation.fieldRobot(oppA, RobotAffiliation.OPPONENT, 0.95, 1.00),
            RobotObservation.fieldRobot(oppB, RobotAffiliation.OPPONENT, 0.95, 1.00)));
    engine.updateAndEvaluate(1.00, RobotInventoryState.empty(), MatchPeriod.TELEOP, 90.0);

    List<TrackedRobot> tracked = engine.getWorldSnapshot().opponents();
    assertEquals(2, tracked.size());

    // Bind lower-Y track to DEFENDER (9999) and higher-Y track to PRIMARY_SCORER (254)
    int defenderTrackId = -1;
    int scorerTrackId = -1;
    for (TrackedRobot r : tracked) {
      if (r.pose3d().getY() < 4.0) {
        defenderTrackId = r.trackId();
      } else {
        scorerTrackId = r.trackId();
      }
    }
    assertTrue(engine.bindOpponentTrackToTeam(defenderTrackId, 9999));
    assertTrue(engine.bindOpponentTrackToTeam(scorerTrackId, 254));

    // Step again and confirm DefenseInterceptor prioritizes Team 254 (PRIMARY_SCORER)
    engine.updateEgoPose(ego, 1.02);
    engine.feedRobots(
        List.of(
            RobotObservation.fieldRobot(oppA, RobotAffiliation.OPPONENT, 0.95, 1.02),
            RobotObservation.fieldRobot(oppB, RobotAffiliation.OPPONENT, 0.95, 1.02)));
    TacticalRecommendation rec =
        engine.updateAndEvaluate(1.02, RobotInventoryState.empty(), MatchPeriod.TELEOP, 89.98);

    assertEquals(EncounterIntent.DEFEND_INTERCEPT, rec.intent());
    assertEquals("opponent:" + scorerTrackId, rec.targetId());
    assertTrue(
        rec.reason().contains("Team 254") && rec.reason().contains("PRIMARY_SCORER"),
        "Expected team & role in defense reason, got: " + rec.reason());
  }

  @Test
  void scoutingPriorsBoostEndgameSpecialistAndResetClearsOnlyTrackBindings() {
    ScoutingPriorRegistry priors = ScoutingPriorRegistry.fromJson(SCOUTING_JSON);
    OpponentProfile endgameBot = priors.findTeamProfile(1678).orElseThrow();
    double teleopWeight = endgameBot.effectiveThreatWeight(MatchPeriod.TELEOP, 90.0, "");
    double endgameWeight = endgameBot.effectiveThreatWeight(MatchPeriod.TELEOP, 20.0, "");
    assertTrue(
        endgameWeight > teleopWeight,
        "ENDGAME_SPECIALIST threat weight should increase during ENDGAME");

    assertTrue(priors.bindTrackToTeam(7, 1678));
    assertTrue(priors.resolveForTrack(7).isPresent());

    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine engine = YaeceEngine.builder(rebuilt).withScoutingPriors(priors).build();
    engine.resetForMatch();

    assertFalse(priors.resolveForTrack(7).isPresent(), "Match reset must clear track bindings");
    assertTrue(
        priors.findTeamProfile(1678).isPresent(),
        "Match reset must retain pre-match team profiles");
  }

  @Test
  void yacSim3dBridgeStepsEngineComputesCgDeratedAlignSotmAndRecordsReplay() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine engine =
        YaeceEngine.builder(rebuilt)
            .withAlliance(AllianceSide.BLUE)
            .withKinematics(RobotKinematicsSpec.FRC_SWERVE_DEFAULT)
            .build();
    PolarAutoAlignController align = new PolarAutoAlignController();
    ShootOnTheMoveSolver sotm =
        new ShootOnTheMoveSolver(ShootOnTheMoveSolver.Config.REBUILT_DEFAULT, d -> 0.15 + 0.18 * d);
    YaeceInputRecorder recorder = new YaeceInputRecorder();
    YacSim3dBridge bridge = new YacSim3dBridge(engine, align, sotm, recorder);

    YacSim3dBridge.YacSimTickOutput lastOut = null;
    for (int i = 0; i < 6; i++) {
      double t = 1.0 + i * 0.02;
      YacSim3dBridge.SimulatedRobotState ego =
          new YacSim3dBridge.SimulatedRobotState(
              new Pose3d(new Translation3d(2.2 + i * 0.02, 4.0, 0.0), new Rotation3d()),
              0.0,
              1.5,
              0.0,
              0.90);
      YacSim3dBridge.YacSimTickInput input =
          new YacSim3dBridge.YacSimTickInput(
              t,
              120.0 - i * 0.02,
              MatchPeriod.TELEOP,
              ego,
              List.of(
                  new YacSim3dBridge.SimulatedPiecePose(
                      0, new Pose3d(6.0, 3.5, 0.075, new Rotation3d()))),
              List.of(
                  new YacSim3dBridge.SimulatedOtherRobot(
                      new Pose3d(12.0, 4.0, 0.0, new Rotation3d()), RobotAffiliation.OPPONENT)),
              RobotInventoryState.holding("fuel", 3),
              Map.of("hub_active", true));
      lastOut = bridge.step(input);
    }

    assertEquals(EncounterIntent.SCORE, lastOut.recommendation().intent());
    assertTrue(lastOut.alignCommand().isPresent(), "Expected polar align command");
    assertTrue(lastOut.sotmSolution().isPresent(), "Expected SOTM solution during SCORE intent");

    double speed =
        Math.hypot(
            lastOut.alignCommand().get().vxFieldMps(), lastOut.alignCommand().get().vyFieldMps());
    assertTrue(speed <= 2.0, "High CG extension (0.90) should derate linear speed, got " + speed);

    // Verify recorded JSONL replays cleanly via YaeceReplayHarness
    List<RecordedTick> parsed = YaeceInputRecorder.parseJsonLines(recorder.toJsonLines());
    assertEquals(6, parsed.size());
    List<TacticalRecommendation> replayed =
        YaeceReplayHarness.replay(
            rebuilt, AllianceSide.BLUE, RobotKinematicsSpec.FRC_SWERVE_DEFAULT, parsed);
    assertEquals(6, replayed.size());
    assertEquals(lastOut.recommendation().intent(), replayed.get(5).intent());
    assertEquals(lastOut.recommendation().targetId(), replayed.get(5).targetId());
  }
}
