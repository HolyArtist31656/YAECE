// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * OctoMap-inspired clamped log-odds occupancy table for live scored-element slot tracking.
 *
 * <p>With default parameters ({@code p_hit = 0.70} → {@code +0.847}, {@code p_miss = 0.40} → {@code
 * -0.405}, clamped to probabilities {@code [0.12, 0.97]}), a saturated occupied slot flips back to
 * free after ~9 consecutive ray misses—preventing stale ghost pieces from lingering in tactical
 * arbitration.
 */
public final class SlotOccupancyGrid {
  public static final double DEFAULT_P_HIT = 0.70;
  public static final double DEFAULT_P_MISS = 0.40;
  public static final double DEFAULT_P_MIN = 0.12;
  public static final double DEFAULT_P_MAX = 0.97;
  public static final double DEFAULT_OCCUPIED_THRESHOLD = 0.65;
  public static final double DEFAULT_FREE_THRESHOLD = 0.35;

  private final double logOddsHit;
  private final double logOddsMiss;
  private final double logOddsMin;
  private final double logOddsMax;
  private final double occupiedThreshold;
  private final double freeThreshold;
  private final Map<String, Double> slotLogOdds = new LinkedHashMap<>();
  private final Map<String, Double> lastUpdateSec = new LinkedHashMap<>();

  public SlotOccupancyGrid() {
    this(
        DEFAULT_P_HIT,
        DEFAULT_P_MISS,
        DEFAULT_P_MIN,
        DEFAULT_P_MAX,
        DEFAULT_OCCUPIED_THRESHOLD,
        DEFAULT_FREE_THRESHOLD);
  }

  public SlotOccupancyGrid(
      double pHit,
      double pMiss,
      double pMin,
      double pMax,
      double occupiedThreshold,
      double freeThreshold) {
    if (pHit <= 0.5 || pHit >= 1.0 || pMiss <= 0.0 || pMiss >= 0.5) {
      throw new IllegalArgumentException("Require 0.5 < pHit < 1.0 and 0.0 < pMiss < 0.5.");
    }
    if (pMin <= 0.0 || pMax >= 1.0 || pMin >= pMax) {
      throw new IllegalArgumentException("Require 0.0 < pMin < pMax < 1.0.");
    }
    this.logOddsHit = probToLogOdds(pHit);
    this.logOddsMiss = probToLogOdds(pMiss);
    this.logOddsMin = probToLogOdds(pMin);
    this.logOddsMax = probToLogOdds(pMax);
    this.occupiedThreshold = occupiedThreshold;
    this.freeThreshold = freeThreshold;
  }

  /** Records a positive occupancy observation (camera ROI, RGB-D, LiDAR, or CANrange hit). */
  public double recordHit(String slotId, double timestampSec) {
    Objects.requireNonNull(slotId, "slotId");
    double next = clamp(slotLogOdds.getOrDefault(slotId, 0.0) + logOddsHit, logOddsMin, logOddsMax);
    slotLogOdds.put(slotId, next);
    lastUpdateSec.put(slotId, timestampSec);
    return logOddsToProb(next);
  }

  /** Records a clear/empty ray observation passing through {@code slotId}. */
  public double recordMiss(String slotId, double timestampSec) {
    Objects.requireNonNull(slotId, "slotId");
    double next =
        clamp(slotLogOdds.getOrDefault(slotId, 0.0) + logOddsMiss, logOddsMin, logOddsMax);
    slotLogOdds.put(slotId, next);
    lastUpdateSec.put(slotId, timestampSec);
    return logOddsToProb(next);
  }

  /**
   * @return Current occupancy probability in {@code [pMin, pMax]} (unknown slots return 0.50).
   */
  public double getProbability(String slotId) {
    return logOddsToProb(slotLogOdds.getOrDefault(slotId, 0.0));
  }

  public boolean isOccupied(String slotId) {
    return getProbability(slotId) >= occupiedThreshold;
  }

  public boolean isFree(String slotId) {
    return getProbability(slotId) <= freeThreshold;
  }

  public double getLastUpdateSec(String slotId) {
    return lastUpdateSec.getOrDefault(slotId, Double.NaN);
  }

  public Set<String> getOccupiedSlotIds() {
    Set<String> out = new LinkedHashSet<>();
    for (Map.Entry<String, Double> e : slotLogOdds.entrySet()) {
      if (logOddsToProb(e.getValue()) >= occupiedThreshold) {
        out.add(e.getKey());
      }
    }
    return Collections.unmodifiableSet(out);
  }

  public void clear() {
    slotLogOdds.clear();
    lastUpdateSec.clear();
  }

  public static double probToLogOdds(double p) {
    return Math.log(p / (1.0 - p));
  }

  public static double logOddsToProb(double l) {
    return 1.0 / (1.0 + Math.exp(-l));
  }

  private static double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(max, v));
  }
}
