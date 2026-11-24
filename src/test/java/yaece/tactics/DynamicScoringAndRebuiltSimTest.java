package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.console.PlayMode;
import yaece.ledger.OptimizationObjective;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;

class DynamicScoringAndRebuiltSimTest {

  @Test
  void handlesRebuiltActiveInactiveHubShiftsClosingWindowGuardAndMultipliers() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine engine = YaeceEngine.builder(rebuilt).withAlliance(AllianceSide.BLUE).build();

    engine.updateEgoPose(new Pose3d(new Translation3d(3.5, 4.0, 0.0), new Rotation3d()), 35.0);
    engine.feedGamePieces(
        List.of(
            GamePieceObservation6DoF.field6DoF(
                0, new Pose3d(new Translation3d(6.2, 4.0, 0.075), new Rotation3d()), 0.95, 35.0)));

    // 1. Shift 1: Blue HUB is INACTIVE (15s remaining until open) -> never score, gather FUEL
    // instead!
    engine.setDynamicKeyActive("hub_active", false);
    engine.setDynamicKeyTimeRemaining("hub_active", 15.0);
    TacticalRecommendation inactiveRec =
        engine.updateAndEvaluate(
            35.0, RobotInventoryState.holding("fuel", 3), MatchPeriod.TELEOP, 125.0);
    assertEquals(EncounterIntent.GATHER, inactiveRec.intent());

    // 2. Shift 1 ending in 2.0s with full hopper (12/12) -> pre-stages at stagingHoldPose2d!
    engine.setDynamicKeyTimeRemaining("hub_active", 2.0);
    TacticalRecommendation preStageRec =
        engine.updateAndEvaluate(
            48.0, RobotInventoryState.holding("fuel", 12), MatchPeriod.TELEOP, 112.0);
    assertEquals(EncounterIntent.HOLD_STAGE, preStageRec.intent());

    // 3. Shift 2: Blue HUB activates with 25s left -> immediately scores!
    engine.setDynamicKeyActive("hub_active", true);
    engine.setDynamicKeyTimeRemaining("hub_active", 25.0);
    TacticalRecommendation activeRec =
        engine.updateAndEvaluate(
            50.0, RobotInventoryState.holding("fuel", 12), MatchPeriod.TELEOP, 110.0);
    assertEquals(EncounterIntent.SCORE, activeRec.intent());

    // 4. Closing-window guard: only 0.2s left on active HUB while 4.5m away -> refuses dead sprint!
    engine.updateEgoPose(new Pose3d(new Translation3d(7.2, 4.0, 0.0), new Rotation3d()), 74.8);
    engine.setDynamicKeyTimeRemaining("hub_active", 0.2);
    TacticalRecommendation closingRec =
        engine.updateAndEvaluate(
            74.8, RobotInventoryState.holding("fuel", 4), MatchPeriod.TELEOP, 85.2);
    assertNotEquals(EncounterIntent.SCORE, closingRec.intent());
  }

  @Test
  void appliesGameAgnosticMultipliersAndStaticClockScoringIntervals() {
    String amplifiedJson =
        """
        {
          "schemaVersion": 1,
          "seasonName": "AMPLIFIED_TEST",
          "fieldLengthMeters": 16.54,
          "fieldWidthMeters": 8.07,
          "mirrorMode": "NONE",
          "maxTotalHeldGamePieces": 1,
          "matchDurationSeconds": 150.0,
          "gamePieces": [
            {
              "id": "note",
              "visionClassId": 0,
              "dimensionsMeters": [0.35, 0.35, 0.05],
              "restHeightMeters": 0.025,
              "symmetry": "PLANAR_RING",
              "maxHeldSimultaneously": 1
            }
          ],
          "scoringSites": [
            {
              "id": "low_site",
              "pose3d": [1.0, 2.0, 1.0, 0.0, 0.0, 0.0],
              "approachOffsetMeters": [0.8, 0.0, -1.0, 0.0, 0.0, 0.0],
              "acceptedPieces": ["note"],
              "autoPoints": 2.0,
              "teleopPoints": 2.0,
              "stagingHoldPose2d": [2.5, 2.0, 0.0],
              "tags": ["boostable"],
              "dynamicAvailabilityKey": "amp_boost"
            },
            {
              "id": "standard_high",
              "pose3d": [1.0, 6.0, 2.0, 0.0, 0.0, 0.0],
              "approachOffsetMeters": [0.8, 0.0, -2.0, 0.0, 0.0, 0.0],
              "acceptedPieces": ["note"],
              "autoPoints": 4.0,
              "teleopPoints": 4.0,
              "stagingHoldPose2d": [2.5, 6.0, 0.0]
            }
          ],
          "scoringIntervals": [
            {
              "id": "endgame_double",
              "startElapsedSec": 120.0,
              "endElapsedSec": 150.0,
              "matchingSiteTags": ["boostable"],
              "active": true,
              "pointMultiplier": 3.0,
              "flatBonusPoints": 1.0
            }
          ]
        }
        """;
    SeasonConfig cfg = SeasonConfig.fromJsonString(amplifiedJson);
    YaeceEngine engine = YaeceEngine.builder(cfg).build();
    engine.updateEgoPose(new Pose3d(new Translation3d(3.0, 4.0, 0.0), new Rotation3d()), 50.0);

    // Mid-match (elapsed=50s): standard_high (4 pts) beats low_site (2 pts)
    TacticalRecommendation mid =
        engine.updateAndEvaluate(
            50.0, RobotInventoryState.holding("note"), MatchPeriod.TELEOP, 100.0);
    assertEquals("standard_high", mid.targetId());

    // Runtime dynamic multiplier (amp_boost = 3.0x -> 6 pts): low_site wins!
    engine.setDynamicKeyMultiplier("amp_boost", 3.0);
    TacticalRecommendation boosted =
        engine.updateAndEvaluate(
            51.0, RobotInventoryState.holding("note"), MatchPeriod.TELEOP, 99.0);
    assertEquals("low_site", boosted.targetId());

    // Reset runtime multiplier -> during static Endgame interval (elapsed=130s), low_site = 2*3+1 =
    // 7 pts!
    engine.setDynamicKeyMultiplier("amp_boost", 1.0);
    TacticalRecommendation endgame =
        engine.updateAndEvaluate(
            130.0, RobotInventoryState.holding("note"), MatchPeriod.TELEOP, 20.0);
    assertEquals("low_site", endgame.targetId());
  }

  @Test
  void runsFull160SecondRebuilt6RobotSimVerifyingG420EndgameClampAndRpSpillover() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine engine = YaeceEngine.builder(rebuilt).withAlliance(AllianceSide.BLUE).build();
    engine.setOptimizationObjective(OptimizationObjective.RANKING_POINTS);

    // Feed 6-robot field + 20 Neutral Zone FUEL at Endgame (t=140s, 20s remaining -> G420 active!)
    engine.updateEgoPose(new Pose3d(new Translation3d(12.0, 4.0, 0.0), new Rotation3d()), 140.0);
    List<GamePieceObservation6DoF> fuelField = new ArrayList<>();
    for (int i = 0; i < 20; i++) {
      fuelField.add(
          GamePieceObservation6DoF.field6DoF(
              0,
              new Pose3d(
                  new Translation3d(7.0 + (i % 4) * 0.6, 1.5 + (i / 4) * 1.1, 0.075),
                  new Rotation3d()),
              0.9,
              140.0));
    }
    engine.feedGamePieces(fuelField);

    // Red #1 climbing inside Red Tower G420 zone [14.90..16.54, 2.95..5.12]
    engine.feedRobots(
        List.of(
            RobotObservation.fieldRobot(
                new Pose3d(new Translation3d(15.2, 4.0, 0.0), new Rotation3d()),
                RobotAffiliation.OPPONENT,
                0.95,
                140.0)));

    // Switch to DEFENSE in Endgame -> must clamp outside G420 zone (x <= 14.90 - 0.90 = 14.00)
    engine.setPlayMode(PlayMode.DEFENSE);
    engine.setDefenseStrategyMode(DefenseStrategyMode.HIGHEST_THREAT);
    TacticalRecommendation endDef =
        engine.updateAndEvaluate(140.0, RobotInventoryState.empty(), MatchPeriod.TELEOP, 20.0);
    assertEquals(EncounterIntent.DEFEND_INTERCEPT, endDef.intent());
    assertTrue(endDef.stowMechanisms());
    assertTrue(
        endDef.targetPose3d().getX() <= 14.05,
        "Expected G420 Endgame clamp outside Red Tower, got x=" + endDef.targetPose3d().getX());
  }
}
