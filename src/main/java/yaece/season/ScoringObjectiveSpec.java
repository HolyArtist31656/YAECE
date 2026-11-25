// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

import java.util.Objects;
import java.util.Set;

/**
 * Immutable threshold or linked scoring objective (bonus match points or Ranking Point) defined in
 * a season JSON file.
 */
public record ScoringObjectiveSpec(
    String id,
    ObjectiveRewardType rewardType,
    double rewardValue,
    ObjectiveAggregationMode aggregationMode,
    int targetCount,
    Set<String> matchingSiteIds,
    Set<String> matchingSiteTags,
    Set<String> matchingPieceIds) {

  public ScoringObjectiveSpec {
    if (id == null || id.isBlank()) {
      throw new IllegalArgumentException("ScoringObjectiveSpec.id must be non-blank");
    }
    Objects.requireNonNull(rewardType, "ScoringObjectiveSpec.rewardType must not be null");
    if (!Double.isFinite(rewardValue) || rewardValue <= 0.0) {
      throw new IllegalArgumentException("ScoringObjectiveSpec.rewardValue must be finite and > 0");
    }
    Objects.requireNonNull(
        aggregationMode, "ScoringObjectiveSpec.aggregationMode must not be null");
    if (targetCount < 1) {
      throw new IllegalArgumentException("ScoringObjectiveSpec.targetCount must be >= 1");
    }
    matchingSiteIds = matchingSiteIds == null ? Set.of() : Set.copyOf(matchingSiteIds);
    matchingSiteTags = matchingSiteTags == null ? Set.of() : Set.copyOf(matchingSiteTags);
    matchingPieceIds = matchingPieceIds == null ? Set.of() : Set.copyOf(matchingPieceIds);
  }

  /** Returns whether a scored piece at {@code site} contributes toward this objective. */
  public boolean matches(ScoringSiteSpec site, String pieceId) {
    Objects.requireNonNull(site, "site must not be null");
    if (!matchingPieceIds.isEmpty() && !matchingPieceIds.contains(pieceId)) {
      return false;
    }
    boolean siteIdFilter = !matchingSiteIds.isEmpty();
    boolean siteTagFilter = !matchingSiteTags.isEmpty();
    if (!siteIdFilter && !siteTagFilter) {
      return true;
    }
    if (siteIdFilter && matchingSiteIds.contains(site.id())) {
      return true;
    }
    if (siteTagFilter) {
      for (String tag : matchingSiteTags) {
        if (site.hasTag(tag)) {
          return true;
        }
      }
    }
    return false;
  }
}
