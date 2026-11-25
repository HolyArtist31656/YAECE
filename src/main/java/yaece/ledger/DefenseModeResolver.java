// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.ledger;

import java.util.Objects;
import yaece.season.DefenseConfigSpec;
import yaece.season.DefenseStrategyMode;

/**
 * Resolves the concrete defensive strategy ({@link DefenseStrategyMode#HIGHEST_THREAT} vs. {@link
 * DefenseStrategyMode#ZONE_DEFENSE}) from driver preference and live/projected score margins.
 */
public final class DefenseModeResolver {
  private static final double MIN_SCORE_CONFIDENCE = 0.25;

  private DefenseModeResolver() {}

  /**
   * Resolves the active concrete defense mode.
   *
   * <p>If the driver explicitly selected {@link DefenseStrategyMode#HIGHEST_THREAT} or {@link
   * DefenseStrategyMode#ZONE_DEFENSE}, that selection is honored unconditionally. When set to
   * {@link DefenseStrategyMode#AUTO_ADAPTIVE}, uses the projected score margin (when confidence
   * &gt;= 0.25): leading by at least {@code zoneDefenseLeadMarginPoints} selects {@code
   * ZONE_DEFENSE} to protect scoring lanes; trailing by at least {@code
   * highestThreatTrailMarginPoints} selects {@code HIGHEST_THREAT} to disrupt the opponent's top
   * scorer; otherwise falls back to {@code defaultPreferredMode}.
   */
  public static DefenseStrategyMode resolve(
      DefenseStrategyMode requestedMode,
      DefenseConfigSpec defenseConfig,
      ScoreProjection projection) {
    Objects.requireNonNull(requestedMode, "requestedMode must not be null");
    Objects.requireNonNull(defenseConfig, "defenseConfig must not be null");
    if (requestedMode != DefenseStrategyMode.AUTO_ADAPTIVE) {
      return requestedMode;
    }
    if (projection == null || projection.opponentProjectionConfidence() < MIN_SCORE_CONFIDENCE) {
      return defenseConfig.defaultPreferredMode();
    }
    double margin = projection.projectedMargin();
    if (margin >= defenseConfig.zoneDefenseLeadMarginPoints()) {
      return DefenseStrategyMode.ZONE_DEFENSE;
    }
    if (margin <= -defenseConfig.highestThreatTrailMarginPoints()) {
      return DefenseStrategyMode.HIGHEST_THREAT;
    }
    return defenseConfig.defaultPreferredMode();
  }
}
