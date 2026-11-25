// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.ballistics;

import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Objects;
import java.util.TreeMap;
import java.util.function.DoubleUnaryOperator;
import yaece.json.MinimalJson;

/**
 * Zero-dependency distance-indexed ballistic interpolation table (supporting Hub scoring vs. Ferry
 * passing tables) loadable from JSON via {@link MinimalJson} and directly compatible with {@link
 * yaece.tactics.ShootOnTheMoveSolver}.
 */
public final class BallisticTable {
  private final String tableName;
  private final NavigableMap<Double, BallisticShotParams> entries;

  private BallisticTable(String tableName, NavigableMap<Double, BallisticShotParams> entries) {
    if (entries.isEmpty()) {
      throw new IllegalArgumentException("BallisticTable must contain at least one entry");
    }
    this.tableName = Objects.requireNonNull(tableName, "tableName must not be null");
    this.entries = new TreeMap<>(entries);
  }

  public static Builder builder(String tableName) {
    return new Builder(tableName);
  }

  public static final class Builder {
    private final String tableName;
    private final TreeMap<Double, BallisticShotParams> entries = new TreeMap<>();

    private Builder(String tableName) {
      this.tableName = Objects.requireNonNull(tableName, "tableName must not be null");
    }

    public Builder put(double distanceMeters, BallisticShotParams params) {
      if (!Double.isFinite(distanceMeters) || distanceMeters < 0.0) {
        throw new IllegalArgumentException("distanceMeters must be finite and >= 0");
      }
      entries.put(distanceMeters, Objects.requireNonNull(params, "params must not be null"));
      return this;
    }

    public BallisticTable build() {
      return new BallisticTable(tableName, entries);
    }
  }

  public String tableName() {
    return tableName;
  }

  public int size() {
    return entries.size();
  }

  public double minDistanceMeters() {
    return entries.firstKey();
  }

  public double maxDistanceMeters() {
    return entries.lastKey();
  }

  /**
   * Returns interpolated {@link BallisticShotParams} at {@code distanceMeters}, clamped at ends.
   */
  public BallisticShotParams get(double distanceMeters) {
    Map.Entry<Double, BallisticShotParams> floor = entries.floorEntry(distanceMeters);
    Map.Entry<Double, BallisticShotParams> ceiling = entries.ceilingEntry(distanceMeters);
    if (floor == null) {
      return ceiling.getValue();
    }
    if (ceiling == null || floor.getKey().equals(ceiling.getKey())) {
      return floor.getValue();
    }
    double t = (distanceMeters - floor.getKey()) / (ceiling.getKey() - floor.getKey());
    return floor.getValue().interpolate(ceiling.getValue(), t);
  }

  /** Returns a {@link DoubleUnaryOperator} mapping distance (m) to empirical time-of-flight (s). */
  public DoubleUnaryOperator tofLookup() {
    return d -> get(d).timeOfFlightSec();
  }

  /**
   * Parses a ballistic LUT from JSON using {@link MinimalJson}.
   *
   * <p>Expected format:
   *
   * <pre>{@code
   * {
   *   "tableName": "rebuilt_hub",
   *   "entries": [
   *     {
   *       "distanceMeters": 1.5,
   *       "flywheelRps": 52.0,
   *       "hoodAngleRad": 0.42,
   *       "timeOfFlightSec": 0.42,
   *       "headingToleranceDeg": 5.0
   *     }
   *   ]
   * }
   * }</pre>
   */
  public static BallisticTable fromJson(String json) {
    Map<String, Object> root = MinimalJson.parseObject(json);
    String name = MinimalJson.optionalString(root, "tableName", "default", "ballistics");
    List<Object> rows = MinimalJson.requireArray(root, "entries", "ballistics");
    Builder b = builder(name);
    for (int i = 0; i < rows.size(); i++) {
      String ctx = "ballistics.entries[" + i + "]";
      Map<String, Object> row = MinimalJson.asObject(rows.get(i), ctx);
      double dist = MinimalJson.requireDouble(row, "distanceMeters", ctx);
      double rps = MinimalJson.requireDouble(row, "flywheelRps", ctx);
      double hood = MinimalJson.requireDouble(row, "hoodAngleRad", ctx);
      double tof = MinimalJson.requireDouble(row, "timeOfFlightSec", ctx);
      double headingTolDeg = MinimalJson.optionalDouble(row, "headingToleranceDeg", 3.5, ctx);
      b.put(dist, new BallisticShotParams(rps, hood, tof, Math.toRadians(headingTolDeg)));
    }
    return b.build();
  }
}
