// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

import java.util.List;
import java.util.Objects;

/**
 * Season-level defaults for defensive strategy selection and score-adaptive switching.
 *
 * @param defaultPreferredMode concrete fallback mode ({@link DefenseStrategyMode#HIGHEST_THREAT} or
 *     {@link DefenseStrategyMode#ZONE_DEFENSE}) used when score confidence is low or tied
 * @param zoneDefenseLeadMarginPoints projected point lead above which {@link
 *     DefenseStrategyMode#AUTO_ADAPTIVE} prefers {@link DefenseStrategyMode#ZONE_DEFENSE} to
 *     protect scoring lanes
 * @param highestThreatTrailMarginPoints projected point deficit below which {@link
 *     DefenseStrategyMode#AUTO_ADAPTIVE} prefers {@link DefenseStrategyMode#HIGHEST_THREAT} to
 *     disrupt the opponent's primary scorer
 * @param defenseZones named patrol corridors for Zone Defense
 */
public record DefenseConfigSpec(
    DefenseStrategyMode defaultPreferredMode,
    double zoneDefenseLeadMarginPoints,
    double highestThreatTrailMarginPoints,
    List<DefenseZoneSpec> defenseZones) {

  public static final DefenseConfigSpec DEFAULT =
      new DefenseConfigSpec(DefenseStrategyMode.HIGHEST_THREAT, 12.0, 6.0, List.of());

  public DefenseConfigSpec {
    Objects.requireNonNull(
        defaultPreferredMode, "DefenseConfigSpec.defaultPreferredMode must not be null");
    if (defaultPreferredMode == DefenseStrategyMode.AUTO_ADAPTIVE) {
      throw new IllegalArgumentException(
          "DefenseConfigSpec.defaultPreferredMode must be HIGHEST_THREAT or ZONE_DEFENSE");
    }
    if (!Double.isFinite(zoneDefenseLeadMarginPoints) || zoneDefenseLeadMarginPoints < 0.0) {
      throw new IllegalArgumentException(
          "DefenseConfigSpec.zoneDefenseLeadMarginPoints must be finite and >= 0");
    }
    if (!Double.isFinite(highestThreatTrailMarginPoints) || highestThreatTrailMarginPoints < 0.0) {
      throw new IllegalArgumentException(
          "DefenseConfigSpec.highestThreatTrailMarginPoints must be finite and >= 0");
    }
    defenseZones = defenseZones == null ? List.of() : List.copyOf(defenseZones);
  }
}
