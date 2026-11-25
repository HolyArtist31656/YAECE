// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.ledger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;

class MatchScoreLedgerTest {
  private SeasonConfig season;
  private MatchScoreLedger ledger;

  @BeforeEach
  void setUp() {
    season = SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    ledger = new MatchScoreLedger(season);
  }

  @Test
  void reconcilesObservedScoreWithoutDoubleCountingPriorSelfScores() {
    // Auto: self scores 1 high_post_left (7 pts) at t=5s and 1 alliance_hub (3 pts) at t=12s -> 10
    // pts
    ledger.recordSelfScored("high_post_left", "cylinder_core", MatchPeriod.AUTO, 5.0);
    ledger.recordSelfScored("alliance_hub", "fuel_sphere", MatchPeriod.AUTO, 12.0);
    assertEquals(10.0, ledger.getEstimatedAllianceScore(), 1e-9);

    // Human console reports observed alliance score = 24 pts at t=20s (includes our 10 pts + 14
    // teammate pts)
    ledger.updateObservedAllianceScore(24.0, 20.0);
    assertEquals(24.0, ledger.getEstimatedAllianceScore(), 1e-9);

    // Self scores again in Teleop at t=35s (high_post_right = 5 pts) AFTER the console timestamp
    ledger.recordSelfScored("high_post_right", "cylinder_core", MatchPeriod.TELEOP, 35.0);
    assertEquals(29.0, ledger.getEstimatedAllianceScore(), 1e-9);
  }

  @Test
  void tracksLinkedObjectivesWithTimestampedConsoleOverrideAndPostObservationSelfPieces() {
    // High post pair bonus requires 2 distinct sites tagged "high_post"
    ledger.recordSelfScored("high_post_left", "cylinder_core", MatchPeriod.AUTO, 6.0);
    assertEquals(1, ledger.getEstimatedObjectiveProgress("high_post_pair_bonus"));
    assertFalse(ledger.isObjectiveComplete("high_post_pair_bonus"));

    ledger.updateTeammateSiteCount("high_post_right", 1, 25.0);
    assertEquals(2, ledger.getEstimatedObjectiveProgress("high_post_pair_bonus"));
    assertTrue(ledger.isObjectiveComplete("high_post_pair_bonus"));

    // Energized RP requires 18 fuel in alliance_hub
    ledger.recordSelfScored("alliance_hub", "fuel_sphere", MatchPeriod.TELEOP, 30.0);
    // Console reports 15 total alliance fuel at t=50s (already includes our t=30s piece)
    ledger.updateObservedAllianceObjectiveCount("energized_rp", 15, 50.0);
    assertEquals(15, ledger.getEstimatedObjectiveProgress("energized_rp"));

    // Self scores 3 more fuel after t=50s -> reaches 18 and completes RP!
    ledger.recordSelfScored("alliance_hub", "fuel_sphere", MatchPeriod.TELEOP, 55.0);
    ledger.recordSelfScored("alliance_hub", "fuel_sphere", MatchPeriod.TELEOP, 60.0);
    ledger.recordSelfScored("alliance_hub", "fuel_sphere", MatchPeriod.TELEOP, 65.0);
    assertEquals(18, ledger.getEstimatedObjectiveProgress("energized_rp"));
    assertTrue(ledger.isObjectiveComplete("energized_rp"));
  }

  @Test
  void extrapolatesOpponentScoreFromTimestampedObservationsAndDrivesAutoAdaptiveDefense() {
    // Opponent scores steadily at ~1.0 pt/sec (20 pts at t=20s, 50 pts at t=50s, 80 pts at t=80s)
    ledger.updateOpponentObservedScore(20.0, 20.0);
    ledger.updateOpponentObservedScore(50.0, 50.0);
    ledger.updateOpponentObservedScore(80.0, 80.0);

    // Our alliance is trailing (40 pts at t=80s -> projected 75 vs opponent projected 150)
    ledger.updateObservedAllianceScore(40.0, 80.0);
    ScoreProjection trailing = ledger.projectScores(80.0);
    assertEquals(1.0, trailing.opponentScoringRatePps(), 1e-3);
    assertEquals(150.0, trailing.projectedOpponentFinalScore(), 0.5);
    assertTrue(trailing.projectedMargin() < -10.0);
    assertEquals(
        DefenseStrategyMode.HIGHEST_THREAT,
        DefenseModeResolver.resolve(
            DefenseStrategyMode.AUTO_ADAPTIVE, season.defenseDefaults(), trailing));

    // Now suppose our alliance surges to 120 pts at t=90s -> projected 200 vs opponent ~150
    ledger.updateObservedAllianceScore(120.0, 90.0);
    ScoreProjection leading = ledger.projectScores(90.0);
    assertTrue(leading.projectedMargin() > season.defenseDefaults().zoneDefenseLeadMarginPoints());
    assertEquals(
        DefenseStrategyMode.ZONE_DEFENSE,
        DefenseModeResolver.resolve(
            DefenseStrategyMode.AUTO_ADAPTIVE, season.defenseDefaults(), leading));

    // Explicit driver override always wins regardless of score
    assertEquals(
        DefenseStrategyMode.HIGHEST_THREAT,
        DefenseModeResolver.resolve(
            DefenseStrategyMode.HIGHEST_THREAT, season.defenseDefaults(), leading));
  }

  @Test
  void estimatesOpponentScoreFromObservedPiecesWhenNoScoreboardEntryExists() {
    // Fuel is worth 2.0 teleop pts; 10 pieces at t=30s (20 pts), 25 pieces at t=60s (50 pts)
    ledger.updateOpponentObservedPieces("fuel_sphere", 10, 30.0);
    ledger.updateOpponentObservedPieces("fuel_sphere", 25, 60.0);

    ScoreProjection proj = ledger.projectScores(60.0);
    assertEquals(50.0, proj.opponentCurrentScore(), 1e-6);
    assertEquals(1.0, proj.opponentScoringRatePps(), 1e-3);
    assertEquals(140.0, proj.projectedOpponentFinalScore(), 0.5);
  }
}
