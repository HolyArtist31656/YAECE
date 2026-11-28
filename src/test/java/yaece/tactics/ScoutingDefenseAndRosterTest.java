// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.Map;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.ledger.ScoreProjection;
import yaece.perception.RobotAffiliation;
import yaece.scouting.MatchRoster;
import yaece.scouting.OpponentProfile;
import yaece.scouting.RobotRolePrior;
import yaece.scouting.ScoutingPriorRegistry;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;
import yaece.world.TrackedRobot;
import yaece.world.WorldSnapshot;

/**
 * Unit and fuzz tests for scouting-driven blocking interpolation, {@link MatchRoster}, weighted
 * preferred scoring sites, and ally corridor keep-out in {@link DefenseInterceptor}.
 */
class ScoutingDefenseAndRosterTest {

  private static final long FUZZ_SEED = 2027218103L;
  private static final int FUZZ_TRIALS = 50;

  @Test
  void parsesExtendedScoutingJsonWithBumperDimsSiteWeightsAndMatchRoster() {
    String json =
        """
        {
          "matchRoster": {
            "ownTeam": 2181,
            "allies": [254, 1678],
            "opponents": [6328, 2910, 3847]
          },
          "teams": [
            {
              "teamNumber": 6328,
              "role": "PRIMARY_SCORER",
              "expectedPointsPerMinute": 42.0,
              "maxSpeedMps": 5.4,
              "defenseSusceptibility": 1.2,
              "bumperLengthMeters": 0.94,
              "bumperWidthMeters": 0.86,
              "preferredSiteIds": ["hub"],
              "preferredSiteWeights": { "hub": 2.2, "depot": 0.6 }
            }
          ]
        }
        """;

    ScoutingPriorRegistry registry = ScoutingPriorRegistry.fromJson(json);
    assertTrue(registry.getMatchRoster().isPresent());
    MatchRoster roster = registry.getMatchRoster().get();
    assertTrue(roster.isOwnTeam(2181));
    assertTrue(roster.isAlly(254));
    assertTrue(roster.isOpponent(6328));
    assertEquals(6, roster.allTeams().size());

    OpponentProfile p6328 = registry.findTeamProfile(6328).orElseThrow();
    assertEquals(0.94, p6328.bumperLengthMeters(), 1e-6);
    assertEquals(0.86, p6328.bumperWidthMeters(), 1e-6);
    assertEquals(2.2, p6328.sitePreferenceMultiplier("hub"), 1e-6);
    assertEquals(0.6, p6328.sitePreferenceMultiplier("depot"), 1e-6);
    assertEquals(1.0, p6328.sitePreferenceMultiplier("unlisted"), 1e-6);

    Map<Integer, OpponentProfile> oppProfiles = roster.resolveOpponentProfiles(registry);
    assertEquals(3, oppProfiles.size());
    OpponentProfile p2910 = oppProfiles.get(2910);
    assertEquals(OpponentProfile.DEFAULT_BUMPER_LENGTH_METERS, p2910.bumperLengthMeters(), 1e-6);
    assertEquals(OpponentProfile.DEFAULT_MAX_SPEED_MPS, p2910.maxSpeedMps(), 1e-6);
  }

  @Test
  void matchRosterRejectsDuplicateOrOverlappingTeams() {
    assertThrows(
        IllegalArgumentException.class,
        () -> MatchRoster.of(2181, List.of(2181, 254), List.of(6328)));
    assertThrows(
        IllegalArgumentException.class,
        () -> MatchRoster.of(2181, List.of(254), List.of(254, 6328)));
    assertThrows(
        IllegalArgumentException.class,
        () -> MatchRoster.of(2181, List.of(254, 1678, 1114), List.of(6328)));
  }

  @Test
  void allyCorridorKeepOutShiftsBlockingPoseClearOfChargingAlly() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    ScoutingPriorRegistry priors = new ScoutingPriorRegistry();
    DefenseInterceptor interceptor =
        new DefenseInterceptor(season, RobotKinematicsSpec.FRC_SWERVE_DEFAULT, priors);

    TrackedRobot opp =
        new TrackedRobot(
            10,
            RobotAffiliation.OPPONENT,
            new Pose3d(new Translation3d(8.0, 4.0, 0.0), Rotation3d.ZERO),
            2.5,
            0.0,
            0.45,
            0.1,
            0.08,
            1.0,
            false,
            50.0);
    WorldSnapshot noAllyWorld =
        new WorldSnapshot(
            50.0,
            new Pose3d(new Translation3d(7.0, 4.0, 0.0), Rotation3d.ZERO),
            0.0,
            0.0,
            List.of(),
            List.of(opp),
            Map.of());

    ScoreProjection proj = new ScoreProjection(50.0, 60.0, 58.0, 1.0, 1.0, 140.0, 138.0, 0.6);
    TacticalRecommendation baseline =
        interceptor.evaluate(
            noAllyWorld,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            90.0,
            DefenseStrategyMode.HIGHEST_THREAT,
            proj);
    assertEquals(EncounterIntent.DEFEND_INTERCEPT, baseline.intent());
    Translation2d baseXy = baseline.targetPose3d().getTranslation().toTranslation2d();

    TrackedRobot ally =
        new TrackedRobot(
            20,
            RobotAffiliation.ALLY,
            new Pose3d(new Translation3d(baseXy.getX() - 0.5, baseXy.getY(), 0.0), Rotation3d.ZERO),
            2.0,
            0.0,
            0.45,
            0.1,
            0.08,
            1.0,
            false,
            50.0);
    WorldSnapshot withAllyWorld =
        new WorldSnapshot(
            50.0,
            new Pose3d(new Translation3d(7.0, 4.0, 0.0), Rotation3d.ZERO),
            0.0,
            0.0,
            List.of(),
            List.of(opp, ally),
            Map.of());

    TacticalRecommendation shifted =
        interceptor.evaluate(
            withAllyWorld,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            90.0,
            DefenseStrategyMode.HIGHEST_THREAT,
            proj);
    assertEquals(EncounterIntent.DEFEND_INTERCEPT, shifted.intent());
    Translation2d shiftedXy = shifted.targetPose3d().getTranslation().toTranslation2d();
    assertTrue(
        shiftedXy.getDistance(baseXy) > 0.3,
        "Expected intercept to shift laterally out of ally corridor (delta="
            + shiftedXy.getDistance(baseXy)
            + ")");
  }

  @Test
  void fuzzSixRobotDefenseEvaluationAlwaysProducesFiniteBoundedTargets() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    ScoutingPriorRegistry priors = new ScoutingPriorRegistry();
    priors.registerTeamProfile(
        new OpponentProfile(
            6328,
            RobotRolePrior.PRIMARY_SCORER,
            40.0,
            1.2,
            5.3,
            1.1,
            List.of(),
            0.92,
            0.88,
            Map.of()));
    priors.bindTrackToTeam(1, 6328);
    DefenseInterceptor interceptor =
        new DefenseInterceptor(season, RobotKinematicsSpec.FRC_SWERVE_DEFAULT, priors);
    ScoreProjection proj = new ScoreProjection(50.0, 60.0, 58.0, 1.0, 1.0, 140.0, 138.0, 0.6);
    SplittableRandom rng = new SplittableRandom(FUZZ_SEED);

    for (int i = 0; i < FUZZ_TRIALS; i++) {
      double ts = 20.0 + i * 0.02;
      List<TrackedRobot> bots =
          List.of(
              new TrackedRobot(
                  1,
                  RobotAffiliation.OPPONENT,
                  new Pose3d(
                      new Translation3d(rng.nextDouble(2.0, 14.0), rng.nextDouble(1.0, 7.0), 0.0),
                      Rotation3d.ZERO),
                  rng.nextDouble(-4.0, 4.0),
                  rng.nextDouble(-4.0, 4.0),
                  0.45,
                  0.1,
                  0.08,
                  1.0,
                  false,
                  ts),
              new TrackedRobot(
                  2,
                  RobotAffiliation.OPPONENT,
                  new Pose3d(
                      new Translation3d(rng.nextDouble(2.0, 14.0), rng.nextDouble(1.0, 7.0), 0.0),
                      Rotation3d.ZERO),
                  rng.nextDouble(-3.0, 3.0),
                  rng.nextDouble(-3.0, 3.0),
                  0.45,
                  0.1,
                  0.08,
                  1.0,
                  false,
                  ts),
              new TrackedRobot(
                  3,
                  RobotAffiliation.ALLY,
                  new Pose3d(
                      new Translation3d(rng.nextDouble(2.0, 14.0), rng.nextDouble(1.0, 7.0), 0.0),
                      Rotation3d.ZERO),
                  rng.nextDouble(-3.0, 3.0),
                  rng.nextDouble(-3.0, 3.0),
                  0.45,
                  0.1,
                  0.08,
                  1.0,
                  false,
                  ts));
      WorldSnapshot world =
          new WorldSnapshot(
              ts,
              new Pose3d(
                  new Translation3d(rng.nextDouble(2.0, 14.0), rng.nextDouble(1.0, 7.0), 0.0),
                  Rotation3d.ZERO),
              0.0,
              0.0,
              List.of(),
              bots,
              Map.of());
      TacticalRecommendation rec =
          interceptor.evaluate(
              world,
              AllianceSide.BLUE,
              MatchPeriod.TELEOP,
              rng.nextDouble(5.0, 130.0),
              DefenseStrategyMode.HIGHEST_THREAT,
              proj);
      assertTrue(Double.isFinite(rec.targetPose3d().getX()));
      assertTrue(Double.isFinite(rec.targetPose3d().getY()));
      assertTrue(
          rec.targetPose3d().getX() >= 0.0
              && rec.targetPose3d().getX() <= season.fieldLengthMeters());
      assertTrue(
          rec.targetPose3d().getY() >= 0.0
              && rec.targetPose3d().getY() <= season.fieldWidthMeters());
    }
  }
}
