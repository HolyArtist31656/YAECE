// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import yaece.season.MatchPeriod;
import yaece.season.ScoringIntervalSpec;
import yaece.season.ScoringSiteSpec;
import yaece.season.SeasonConfig;

/**
 * Game-agnostic runtime manager for dynamic scoring availability, timed shift windows, and variable
 * score multipliers / flat bonuses (e.g., 2026 REBUILT Active/Inactive HUB shifts, 2024 CRESCENDO
 * Amplified Speaker bonuses, or Endgame multipliers).
 *
 * <p>Completely optional: if a season does not use dynamic keys or scoring intervals, every site
 * defaults to {@code active = true, multiplier = 1.0, flatBonus = 0.0}.
 */
public final class DynamicScoringState {

  /** Evaluated effective scoring parameters for one site at a given instant. */
  public record EffectiveSiteScoring(
      boolean active,
      double effectiveBasePoints,
      double activeWindowRemainingSec,
      double inactiveTimeUntilOpenSec) {}

  private static final class KeyState {
    boolean active = true;
    double multiplier = 1.0;
    double flatBonusPoints = 0.0;
    double timeRemainingSec = Double.POSITIVE_INFINITY;
  }

  private final Map<String, KeyState> keys = new HashMap<>();

  /**
   * Sets whether a named dynamic availability key (e.g., {@code "hub_active"}) is currently active.
   */
  public void setKeyActive(String key, boolean active) {
    stateFor(key).active = active;
  }

  /** Sets a multiplicative point modifier for a named key (default 1.0). */
  public void setKeyMultiplier(String key, double multiplier) {
    if (!Double.isFinite(multiplier) || multiplier < 0.0) {
      throw new IllegalArgumentException("multiplier must be finite and >= 0");
    }
    stateFor(key).multiplier = multiplier;
  }

  /** Sets a flat additive bonus per scored piece for a named key (default 0.0). */
  public void setKeyBonusPoints(String key, double flatBonusPoints) {
    if (!Double.isFinite(flatBonusPoints)) {
      throw new IllegalArgumentException("flatBonusPoints must be finite");
    }
    stateFor(key).flatBonusPoints = flatBonusPoints;
  }

  /**
   * Sets the remaining duration (seconds) of the current active/inactive state for {@code key},
   * enabling YAECE's closing-window guard and pre-reactivation staging. Pass {@link
   * Double#POSITIVE_INFINITY} if untimed.
   */
  public void setKeyTimeRemainingSec(String key, double secondsRemaining) {
    if (Double.isNaN(secondsRemaining) || secondsRemaining < 0.0) {
      throw new IllegalArgumentException("secondsRemaining must be >= 0");
    }
    stateFor(key).timeRemainingSec = secondsRemaining;
  }

  /**
   * Computes the effective active status, modified base points, and remaining window duration for
   * {@code site}, combining static {@link SeasonConfig#scoringIntervals()} and runtime dynamic
   * keys.
   */
  public EffectiveSiteScoring evaluateSite(
      SeasonConfig season, ScoringSiteSpec site, MatchPeriod period, double elapsedMatchSec) {
    Objects.requireNonNull(season, "season must not be null");
    Objects.requireNonNull(site, "site must not be null");

    boolean active = true;
    double mult = 1.0;
    double flatBonus = 0.0;
    double activeRem = Double.POSITIVE_INFINITY;
    double inactiveRem = Double.POSITIVE_INFINITY;

    // 1. Apply any matching static clock-based ScoringIntervalSpec from SeasonConfig
    for (ScoringIntervalSpec interval : season.scoringIntervals()) {
      if (interval.isActiveAt(elapsedMatchSec) && interval.matchesSite(site)) {
        if (!interval.active()) {
          active = false;
          inactiveRem = Math.min(inactiveRem, interval.endElapsedSec() - elapsedMatchSec);
        } else {
          mult *= interval.pointMultiplier();
          flatBonus += interval.flatBonusPoints();
          activeRem = Math.min(activeRem, interval.endElapsedSec() - elapsedMatchSec);
        }
      }
    }

    // 2. Apply runtime dynamicAvailabilityKey (if set on this site and configured at runtime)
    String key = site.dynamicAvailabilityKey();
    if (!key.isEmpty() && keys.containsKey(key)) {
      KeyState ks = keys.get(key);
      if (!ks.active) {
        active = false;
        inactiveRem = Math.min(inactiveRem, ks.timeRemainingSec);
      } else {
        mult *= ks.multiplier;
        flatBonus += ks.flatBonusPoints;
        activeRem = Math.min(activeRem, ks.timeRemainingSec);
      }
    }

    double base = site.pointsFor(period);
    double effectivePts = active ? Math.max(0.0, base * mult + flatBonus) : 0.0;
    if (effectivePts <= 0.0) {
      active = false;
    }
    return new EffectiveSiteScoring(active, effectivePts, activeRem, inactiveRem);
  }

  public void reset() {
    keys.clear();
  }

  private KeyState stateFor(String key) {
    if (key == null || key.isBlank()) {
      throw new IllegalArgumentException("dynamic key must be non-blank");
    }
    return keys.computeIfAbsent(key, k -> new KeyState());
  }
}
