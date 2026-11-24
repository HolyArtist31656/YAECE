package yaece.season;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;

class SeasonConfigTest {

  @Test
  void loadsExtendedTemplateWithPointSplitsPinRulesZonesObjectivesAndDefense() {
    SeasonConfig config =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");

    assertEquals(1, config.schemaVersion());
    assertEquals("FRC_2027_TEMPLATE", config.seasonName());
    assertEquals(150.0, config.matchDurationSeconds(), 1e-9);
    assertEquals(3.0, config.pinRules().maxPinSeconds(), 1e-9);
    assertEquals(2.2, config.pinRules().backoffTriggerSeconds(), 1e-9);
    assertEquals(1.83, config.pinRules().separationDistanceMeters(), 1e-9);

    ScoringSiteSpec leftPost = config.findScoringSite("high_post_left").orElseThrow();
    assertEquals(7.0, leftPost.pointsFor(MatchPeriod.AUTO), 1e-9);
    assertEquals(5.0, leftPost.pointsFor(MatchPeriod.TELEOP), 1e-9);
    assertTrue(leftPost.hasTag("high_post"));

    Pose3d redPost = config.resolveSitePose(leftPost, AllianceSide.RED);
    assertEquals(16.54 - 1.85, redPost.getX(), 1e-9);
    assertEquals(8.07 - 5.50, redPost.getY(), 1e-9);

    RestrictedZoneSpec tower = config.findRestrictedZone("opponent_tower_endgame").orElseThrow();
    assertEquals(ZoneRuleType.NO_OPPONENT_CONTACT, tower.ruleType());
    assertFalse(tower.isActive(MatchPeriod.AUTO, 140.0));
    assertFalse(tower.isActive(MatchPeriod.TELEOP, 45.0));
    assertTrue(tower.isActive(MatchPeriod.TELEOP, 25.0));
    assertTrue(
        tower.containsPoint(
            new Translation2d(15.0, 4.0),
            AllianceSide.BLUE,
            config.fieldLengthMeters(),
            config.fieldWidthMeters(),
            config.mirrorMode()));

    ScoringObjectiveSpec rp = config.findScoringObjective("energized_rp").orElseThrow();
    assertEquals(ObjectiveRewardType.RANKING_POINT, rp.rewardType());
    assertEquals(18, rp.targetCount());

    assertEquals(
        DefenseStrategyMode.HIGHEST_THREAT, config.defenseDefaults().defaultPreferredMode());
    assertEquals(1, config.defenseDefaults().defenseZones().size());
  }

  @Test
  void supportsBackwardCompatibleSinglePointsField() {
    String legacyJson =
        """
        {
          "schemaVersion": 1,
          "seasonName": "LEGACY_PTS",
          "fieldLengthMeters": 16.54,
          "fieldWidthMeters": 8.07,
          "mirrorMode": "NONE",
          "maxTotalHeldGamePieces": 1,
          "gamePieces": [
            {
              "id": "ball",
              "visionClassId": 0,
              "dimensionsMeters": [0.2, 0.2, 0.2],
              "restHeightMeters": 0.1,
              "symmetry": "SPHERICAL",
              "maxHeldSimultaneously": 1
            }
          ],
          "scoringSites": [
            {
              "id": "goal",
              "pose3d": [1.0, 4.0, 1.0, 0.0, 0.0, 0.0],
              "approachOffsetMeters": [0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
              "acceptedPieces": ["ball"],
              "points": 4.0,
              "stagingHoldPose2d": [2.5, 4.0, 0.0]
            }
          ]
        }
        """;
    SeasonConfig cfg = SeasonConfig.fromJsonString(legacyJson);
    ScoringSiteSpec goal = cfg.findScoringSite("goal").orElseThrow();
    assertEquals(4.0, goal.pointsFor(MatchPeriod.AUTO), 1e-9);
    assertEquals(4.0, goal.pointsFor(MatchPeriod.TELEOP), 1e-9);
    assertEquals(PinRuleSpec.FRC_DEFAULT, cfg.pinRules());
  }

  @Test
  void rejectsUnknownSiteReferenceInScoringObjective() {
    String badJson =
        """
        {
          "schemaVersion": 1,
          "seasonName": "BAD_OBJ",
          "fieldLengthMeters": 16.54,
          "fieldWidthMeters": 8.07,
          "mirrorMode": "NONE",
          "maxTotalHeldGamePieces": 1,
          "gamePieces": [
            {
              "id": "ball",
              "visionClassId": 0,
              "dimensionsMeters": [0.2, 0.2, 0.2],
              "restHeightMeters": 0.1,
              "symmetry": "SPHERICAL",
              "maxHeldSimultaneously": 1
            }
          ],
          "scoringObjectives": [
            {
              "id": "bad_rp",
              "rewardType": "RANKING_POINT",
              "rewardValue": 1.0,
              "aggregationMode": "TOTAL_COUNT",
              "targetCount": 5,
              "matchingSiteIds": ["does_not_exist"]
            }
          ]
        }
        """;
    SeasonConfigException ex =
        assertThrows(SeasonConfigException.class, () -> SeasonConfig.fromJsonString(badJson));
    assertTrue(ex.getMessage().contains("does_not_exist"));
  }
}
