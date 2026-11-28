// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.scouting;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import yaece.json.MinimalJson;
import yaece.season.MatchPeriod;

/**
 * Thread-safe registry of pre-match scouting profiles, active {@link MatchRoster}, and live
 * track-to-team bindings.
 *
 * <p>Teams can load pre-match scouting JSON (generated from pit scouting, Statbotics, or TBA) using
 * {@link #fromJson(String)} with zero third-party JSON dependencies, and bind live {@code
 * TrackedRobot} IDs to team numbers during a match. Unbound tracks default cleanly to neutral
 * weight {@code 1.0} and {@link OpponentProfile#unknownDefaults(int)}.
 */
public final class ScoutingPriorRegistry {
  private static final OpponentProfile ANONYMOUS_DEFAULT = OpponentProfile.unknownDefaults(0);

  private final Map<Integer, OpponentProfile> profilesByTeam = new ConcurrentHashMap<>();
  private final Map<Integer, OpponentProfile> profilesByTrack = new ConcurrentHashMap<>();
  private volatile MatchRoster activeMatchRoster;

  /** Creates an empty {@link ScoutingPriorRegistry}. */
  public ScoutingPriorRegistry() {}

  /**
   * Registers or replaces a pre-match scouting profile indexed by FRC team number.
   *
   * @param profile Scouting profile with {@code teamNumber() > 0}.
   * @return This registry for chaining.
   */
  public ScoutingPriorRegistry registerTeamProfile(OpponentProfile profile) {
    Objects.requireNonNull(profile, "profile must not be null");
    if (profile.teamNumber() <= 0) {
      throw new IllegalArgumentException("teamNumber must be > 0 to register by team");
    }
    profilesByTeam.put(profile.teamNumber(), profile);
    return this;
  }

  /**
   * Sets the active 6-robot {@link MatchRoster} for the current match.
   *
   * @param roster Validated {@link MatchRoster}.
   * @return This registry for chaining.
   */
  public ScoutingPriorRegistry setMatchRoster(MatchRoster roster) {
    this.activeMatchRoster = Objects.requireNonNull(roster, "roster must not be null");
    return this;
  }

  /**
   * Returns the active {@link MatchRoster}, if configured.
   *
   * @return Active {@link MatchRoster}, or empty when unset.
   */
  public Optional<MatchRoster> getMatchRoster() {
    return Optional.ofNullable(activeMatchRoster);
  }

  /** Clears the active {@link MatchRoster} while keeping pre-match team profiles intact. */
  public void clearMatchRoster() {
    this.activeMatchRoster = null;
  }

  /**
   * Binds a live {@code TrackedRobot} track ID to a previously registered FRC team number.
   *
   * @param trackId Live tracker ID.
   * @param teamNumber Registered FRC team number.
   * @return True if a profile existed for {@code teamNumber} and was bound.
   */
  public boolean bindTrackToTeam(int trackId, int teamNumber) {
    OpponentProfile profile = profilesByTeam.get(teamNumber);
    if (profile == null) {
      return false;
    }
    profilesByTrack.put(trackId, profile);
    return true;
  }

  /**
   * Binds a live track ID directly to an {@link OpponentProfile}.
   *
   * @param trackId Live tracker ID.
   * @param profile Profile to associate with {@code trackId}.
   */
  public void bindTrackProfile(int trackId, OpponentProfile profile) {
    profilesByTrack.put(trackId, Objects.requireNonNull(profile, "profile must not be null"));
  }

  /**
   * Returns the profile registered for a team number, if present.
   *
   * @param teamNumber FRC team number.
   * @return Matching profile, if registered.
   */
  public Optional<OpponentProfile> findTeamProfile(int teamNumber) {
    return Optional.ofNullable(profilesByTeam.get(teamNumber));
  }

  /**
   * Returns the profile bound to a live track ID, if present.
   *
   * @param trackId Live tracker ID.
   * @return Bound profile, if present.
   */
  public Optional<OpponentProfile> resolveForTrack(int trackId) {
    return Optional.ofNullable(profilesByTrack.get(trackId));
  }

  /**
   * Returns the profile bound to {@code trackId}, or {@link OpponentProfile#unknownDefaults(int)}
   * when unbound so callers always have valid bumper dimensions and max speed estimates.
   *
   * @param trackId Live tracker ID.
   * @return Bound profile or conservative anonymous defaults.
   */
  public OpponentProfile resolveOrDefaultForTrack(int trackId) {
    return profilesByTrack.getOrDefault(trackId, ANONYMOUS_DEFAULT);
  }

  /**
   * Returns the effective defensive threat multiplier for {@code trackId}, or {@code 1.0} if no
   * scouting profile is bound.
   *
   * @param trackId Live tracker ID.
   * @param period Current match period.
   * @param matchTimeRemainingSec Remaining match time in seconds.
   * @param inferredSiteId Inferred target scoring site ID.
   * @return Effective defensive threat multiplier.
   */
  public double threatMultiplierFor(
      int trackId, MatchPeriod period, double matchTimeRemainingSec, String inferredSiteId) {
    OpponentProfile profile = profilesByTrack.get(trackId);
    if (profile == null) {
      return 1.0;
    }
    return profile.effectiveThreatWeight(period, matchTimeRemainingSec, inferredSiteId);
  }

  /** Clears match-scoped track bindings while preserving pre-match team profiles. */
  public void clearTrackBindings() {
    profilesByTrack.clear();
  }

  /**
   * Returns the number of registered pre-match team profiles.
   *
   * @return Count of registered team profiles.
   */
  public int registeredTeamCount() {
    return profilesByTeam.size();
  }

  /**
   * Parses a pre-match scouting JSON document using YAECE's zero-dependency {@link MinimalJson}.
   *
   * <p>Supports optional {@code bumperLengthMeters}, {@code bumperWidthMeters}, {@code
   * preferredSiteWeights}, and top-level {@code matchRoster} fields in addition to base profile
   * fields.
   *
   * @param json Pre-match scouting JSON string.
   * @return Populated {@link ScoutingPriorRegistry}.
   */
  public static ScoutingPriorRegistry fromJson(String json) {
    Objects.requireNonNull(json, "json must not be null");
    ScoutingPriorRegistry registry = new ScoutingPriorRegistry();
    Map<String, Object> root = MinimalJson.parseObject(json);
    List<Object> teams = MinimalJson.optionalArray(root, "teams", "scouting");
    for (int i = 0; i < teams.size(); i++) {
      String ctx = "scouting.teams[" + i + "]";
      Map<String, Object> entry = MinimalJson.asObject(teams.get(i), ctx);
      int teamNum = MinimalJson.requireInt(entry, "teamNumber", ctx);
      String roleStr = MinimalJson.optionalString(entry, "role", "UNKNOWN", ctx);
      RobotRolePrior role;
      try {
        role = RobotRolePrior.valueOf(roleStr.trim().toUpperCase());
      } catch (IllegalArgumentException ex) {
        role = RobotRolePrior.UNKNOWN;
      }
      double ppm =
          MinimalJson.optionalDouble(
              entry,
              "expectedPointsPerMinute",
              OpponentProfile.DEFAULT_EXPECTED_POINTS_PER_MINUTE,
              ctx);
      double siteMult =
          MinimalJson.optionalDouble(
              entry,
              "preferredSiteThreatMultiplier",
              OpponentProfile.DEFAULT_SITE_THREAT_MULTIPLIER,
              ctx);
      double maxSpeed =
          MinimalJson.optionalDouble(
              entry, "maxSpeedMps", OpponentProfile.DEFAULT_MAX_SPEED_MPS, ctx);
      double susceptibility =
          MinimalJson.optionalDouble(
              entry, "defenseSusceptibility", OpponentProfile.DEFAULT_DEFENSE_SUSCEPTIBILITY, ctx);
      List<String> sites = MinimalJson.optionalStringList(entry, "preferredSiteIds", ctx);
      double bumperLen =
          MinimalJson.optionalDouble(
              entry, "bumperLengthMeters", OpponentProfile.DEFAULT_BUMPER_LENGTH_METERS, ctx);
      double bumperWid =
          MinimalJson.optionalDouble(
              entry, "bumperWidthMeters", OpponentProfile.DEFAULT_BUMPER_WIDTH_METERS, ctx);
      Map<String, Double> siteWeights = parseSiteWeights(entry, ctx);
      registry.registerTeamProfile(
          new OpponentProfile(
              teamNum,
              role,
              ppm,
              siteMult,
              maxSpeed,
              susceptibility,
              sites,
              bumperLen,
              bumperWid,
              siteWeights));
    }
    if (root.containsKey("matchRoster") && root.get("matchRoster") != null) {
      Map<String, Object> rosterObj =
          MinimalJson.asObject(root.get("matchRoster"), "scouting.matchRoster");
      registry.setMatchRoster(MatchRoster.fromJsonMap(rosterObj, "scouting.matchRoster"));
    }
    return registry;
  }

  private static Map<String, Double> parseSiteWeights(Map<String, Object> entry, String ctx) {
    Object raw = entry.get("preferredSiteWeights");
    if (raw == null) {
      return Map.of();
    }
    Map<String, Object> obj = MinimalJson.asObject(raw, ctx + ".preferredSiteWeights");
    Map<String, Double> out = new LinkedHashMap<>(obj.size());
    for (Map.Entry<String, Object> kv : obj.entrySet()) {
      if (!(kv.getValue() instanceof Number num)) {
        throw new IllegalArgumentException(
            ctx + ".preferredSiteWeights." + kv.getKey() + " must be a number");
      }
      out.put(kv.getKey(), num.doubleValue());
    }
    return out;
  }
}
