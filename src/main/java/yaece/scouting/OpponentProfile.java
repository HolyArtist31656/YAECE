// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.scouting;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import yaece.season.MatchPeriod;

/**
 * Immutable pre-match capability profile for a scouted FRC team or tracked robot.
 *
 * @param teamNumber FRC team number (0 when track-only or anonymous).
 * @param rolePrior Scouted tactical role archetype.
 * @param expectedPointsPerMinute Historical or EPA-derived scoring rate (points/min).
 * @param preferredSiteThreatMultiplier Additional multiplier applied to base threat (&gt; 0).
 * @param maxSpeedMps Estimated top drivetrain speed (m/s).
 * @param defenseSusceptibility How strongly physical defense degrades this robot's cycle rate
 *     (typically 0.5 to 1.8).
 * @param preferredSiteIds Optional list of {@code ScoringSiteSpec} IDs this robot favors.
 * @param bumperLengthMeters Outer bumper length in meters (default {@value
 *     #DEFAULT_BUMPER_LENGTH_METERS} m / ~35 in).
 * @param bumperWidthMeters Outer bumper width in meters (default {@value
 *     #DEFAULT_BUMPER_WIDTH_METERS} m / ~35 in).
 * @param preferredSiteWeights Optional per-site preference weights (&gt; 0) keyed by {@code
 *     ScoringSiteSpec} ID; overrides the flat preferred-site bonus when present for a site.
 */
public record OpponentProfile(
    int teamNumber,
    RobotRolePrior rolePrior,
    double expectedPointsPerMinute,
    double preferredSiteThreatMultiplier,
    double maxSpeedMps,
    double defenseSusceptibility,
    List<String> preferredSiteIds,
    double bumperLengthMeters,
    double bumperWidthMeters,
    Map<String, Double> preferredSiteWeights) {

  /** Typical FRC robot outer bumper length in meters (~35 in). */
  public static final double DEFAULT_BUMPER_LENGTH_METERS = 0.89;

  /** Typical FRC robot outer bumper width in meters (~35 in). */
  public static final double DEFAULT_BUMPER_WIDTH_METERS = 0.89;

  /** Default estimated drivetrain top speed in m/s when un-scouted. */
  public static final double DEFAULT_MAX_SPEED_MPS = 4.5;

  /** Default expected scoring rate in points/min when un-scouted. */
  public static final double DEFAULT_EXPECTED_POINTS_PER_MINUTE = 20.0;

  /** Default base preferred-site threat multiplier. */
  public static final double DEFAULT_SITE_THREAT_MULTIPLIER = 1.0;

  /** Default defense susceptibility multiplier. */
  public static final double DEFAULT_DEFENSE_SUSCEPTIBILITY = 1.0;

  private static final double MIN_THREAT_WEIGHT = 0.2;
  private static final double MAX_THREAT_WEIGHT = 4.0;
  private static final double BASELINE_RATE_PPM = 20.0;
  private static final double MIN_RATE_FACTOR = 0.6;
  private static final double MAX_RATE_FACTOR = 1.8;
  private static final double PREFERRED_SITE_LIST_BONUS = 1.35;
  private static final double ENDGAME_WINDOW_SEC = 30.0;
  private static final double ENDGAME_SPECIALIST_BONUS = 1.5;

  /** Compact constructor validating all profile parameters. */
  public OpponentProfile {
    if (teamNumber < 0) {
      throw new IllegalArgumentException("teamNumber must be >= 0");
    }
    Objects.requireNonNull(rolePrior, "rolePrior must not be null");
    if (!Double.isFinite(expectedPointsPerMinute) || expectedPointsPerMinute < 0.0) {
      throw new IllegalArgumentException("expectedPointsPerMinute must be finite and >= 0");
    }
    if (!Double.isFinite(preferredSiteThreatMultiplier) || preferredSiteThreatMultiplier <= 0.0) {
      throw new IllegalArgumentException("preferredSiteThreatMultiplier must be finite and > 0");
    }
    if (!Double.isFinite(maxSpeedMps) || maxSpeedMps <= 0.0) {
      throw new IllegalArgumentException("maxSpeedMps must be finite and > 0");
    }
    if (!Double.isFinite(defenseSusceptibility) || defenseSusceptibility <= 0.0) {
      throw new IllegalArgumentException("defenseSusceptibility must be finite and > 0");
    }
    if (!Double.isFinite(bumperLengthMeters) || bumperLengthMeters <= 0.0) {
      throw new IllegalArgumentException("bumperLengthMeters must be finite and > 0");
    }
    if (!Double.isFinite(bumperWidthMeters) || bumperWidthMeters <= 0.0) {
      throw new IllegalArgumentException("bumperWidthMeters must be finite and > 0");
    }
    preferredSiteIds = preferredSiteIds == null ? List.of() : List.copyOf(preferredSiteIds);
    if (preferredSiteWeights == null || preferredSiteWeights.isEmpty()) {
      preferredSiteWeights = Map.of();
    } else {
      Map<String, Double> copy = new LinkedHashMap<>(preferredSiteWeights.size());
      for (Map.Entry<String, Double> entry : preferredSiteWeights.entrySet()) {
        String key = Objects.requireNonNull(entry.getKey(), "siteId must not be null");
        Double val = entry.getValue();
        if (val == null || !Double.isFinite(val) || val <= 0.0) {
          throw new IllegalArgumentException(
              "preferredSiteWeights['" + key + "'] must be finite and > 0");
        }
        copy.put(key, val);
      }
      preferredSiteWeights = Collections.unmodifiableMap(copy);
    }
  }

  /**
   * Backward-compatible 7-argument constructor defaulting outer bumper dimensions to {@value
   * #DEFAULT_BUMPER_LENGTH_METERS} m x {@value #DEFAULT_BUMPER_WIDTH_METERS} m and empty per-site
   * weights.
   *
   * @param teamNumber FRC team number (0 when track-only or anonymous).
   * @param rolePrior Scouted tactical role archetype.
   * @param expectedPointsPerMinute Historical or EPA-derived scoring rate (points/min).
   * @param preferredSiteThreatMultiplier Additional multiplier applied to base threat (&gt; 0).
   * @param maxSpeedMps Estimated top drivetrain speed (m/s).
   * @param defenseSusceptibility Defense susceptibility multiplier (&gt; 0).
   * @param preferredSiteIds Optional list of preferred {@code ScoringSiteSpec} IDs.
   */
  public OpponentProfile(
      int teamNumber,
      RobotRolePrior rolePrior,
      double expectedPointsPerMinute,
      double preferredSiteThreatMultiplier,
      double maxSpeedMps,
      double defenseSusceptibility,
      List<String> preferredSiteIds) {
    this(
        teamNumber,
        rolePrior,
        expectedPointsPerMinute,
        preferredSiteThreatMultiplier,
        maxSpeedMps,
        defenseSusceptibility,
        preferredSiteIds,
        DEFAULT_BUMPER_LENGTH_METERS,
        DEFAULT_BUMPER_WIDTH_METERS,
        Map.of());
  }

  /**
   * Creates a default profile for a team number and role archetype.
   *
   * @param teamNumber FRC team number (0 when anonymous).
   * @param rolePrior Scouted tactical role archetype.
   * @return Default {@link OpponentProfile}.
   */
  public static OpponentProfile of(int teamNumber, RobotRolePrior rolePrior) {
    return new OpponentProfile(
        teamNumber,
        rolePrior,
        DEFAULT_EXPECTED_POINTS_PER_MINUTE,
        DEFAULT_SITE_THREAT_MULTIPLIER,
        DEFAULT_MAX_SPEED_MPS,
        DEFAULT_DEFENSE_SUSCEPTIBILITY,
        List.of(),
        DEFAULT_BUMPER_LENGTH_METERS,
        DEFAULT_BUMPER_WIDTH_METERS,
        Map.of());
  }

  /**
   * Creates a conservative default profile for an un-scouted robot ({@link RobotRolePrior#UNKNOWN},
   * {@value #DEFAULT_BUMPER_LENGTH_METERS} m x {@value #DEFAULT_BUMPER_WIDTH_METERS} m bumpers,
   * {@value #DEFAULT_MAX_SPEED_MPS} m/s max speed).
   *
   * @param teamNumber FRC team number (0 when anonymous).
   * @return Un-scouted default {@link OpponentProfile}.
   */
  public static OpponentProfile unknownDefaults(int teamNumber) {
    return of(teamNumber, RobotRolePrior.UNKNOWN);
  }

  /**
   * Returns the circumscribing outer bumper radius in meters ({@code 0.5 * hypot(length, width)}).
   *
   * @return Circumscribing bumper radius in meters.
   */
  public double bumperRadiusMeters() {
    return 0.5 * Math.hypot(bumperLengthMeters, bumperWidthMeters);
  }

  /**
   * Returns the scouting preference multiplier for {@code siteId}: uses {@link
   * #preferredSiteWeights()} if configured for {@code siteId}, falls back to {@value
   * #PREFERRED_SITE_LIST_BONUS} if {@code siteId} is in {@link #preferredSiteIds()}, or {@code
   * 1.0} otherwise.
   *
   * @param siteId Scoring site identifier (may be null or empty).
   * @return Positive preference multiplier for {@code siteId}.
   */
  public double sitePreferenceMultiplier(String siteId) {
    if (siteId == null || siteId.isEmpty()) {
      return 1.0;
    }
    Double explicitWeight = preferredSiteWeights.get(siteId);
    if (explicitWeight != null) {
      return explicitWeight;
    }
    return preferredSiteIds.contains(siteId) ? PREFERRED_SITE_LIST_BONUS : 1.0;
  }

  /**
   * Computes the combined defensive threat multiplier for this opponent during {@code period} when
   * approaching {@code inferredSiteId}.
   *
   * @param period Current match period.
   * @param matchTimeRemainingSec Remaining match time in seconds.
   * @param inferredSiteId Inferred target scoring site ID (may be empty).
   * @return Clamped defensive threat multiplier in [{@value #MIN_THREAT_WEIGHT}, {@value
   *     #MAX_THREAT_WEIGHT}].
   */
  public double effectiveThreatWeight(
      MatchPeriod period, double matchTimeRemainingSec, String inferredSiteId) {
    double weight =
        rolePrior.baseThreatMultiplier() * preferredSiteThreatMultiplier * defenseSusceptibility;

    if (expectedPointsPerMinute > 0.0) {
      double rateFactor =
          Math.max(
              MIN_RATE_FACTOR,
              Math.min(MAX_RATE_FACTOR, expectedPointsPerMinute / BASELINE_RATE_PPM));
      weight *= rateFactor;
    }

    weight *= sitePreferenceMultiplier(inferredSiteId);

    if (period == MatchPeriod.TELEOP
        && matchTimeRemainingSec <= ENDGAME_WINDOW_SEC
        && rolePrior == RobotRolePrior.ENDGAME_SPECIALIST) {
      weight *= ENDGAME_SPECIALIST_BONUS;
    }

    return Math.max(MIN_THREAT_WEIGHT, Math.min(MAX_THREAT_WEIGHT, weight));
  }
}
