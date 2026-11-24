package yaece.scouting;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import yaece.json.MinimalJson;
import yaece.season.MatchPeriod;

/**
 * Thread-safe registry of pre-match opponent scouting profiles and live track-to-team bindings.
 *
 * <p>Teams can load pre-match scouting JSON (generated from pit scouting, Statbotics, or TBA) using
 * {@link #fromJson(String)} with zero third-party JSON dependencies, and bind live {@code
 * TrackedRobot} IDs to team numbers during a match. Unbound tracks default cleanly to neutral
 * weight {@code 1.0}.
 */
public final class ScoutingPriorRegistry {
  private final Map<Integer, OpponentProfile> profilesByTeam = new ConcurrentHashMap<>();
  private final Map<Integer, OpponentProfile> profilesByTrack = new ConcurrentHashMap<>();

  public ScoutingPriorRegistry() {}

  /** Registers or replaces a pre-match scouting profile indexed by FRC team number. */
  public ScoutingPriorRegistry registerTeamProfile(OpponentProfile profile) {
    Objects.requireNonNull(profile, "profile must not be null");
    if (profile.teamNumber() <= 0) {
      throw new IllegalArgumentException("teamNumber must be > 0 to register by team");
    }
    profilesByTeam.put(profile.teamNumber(), profile);
    return this;
  }

  /**
   * Binds a live {@code TrackedRobot} track ID to a previously registered FRC team number.
   *
   * @return true if a profile existed for {@code teamNumber} and was bound
   */
  public boolean bindTrackToTeam(int trackId, int teamNumber) {
    OpponentProfile profile = profilesByTeam.get(teamNumber);
    if (profile == null) {
      return false;
    }
    profilesByTrack.put(trackId, profile);
    return true;
  }

  /** Binds a live track ID directly to an {@link OpponentProfile}. */
  public void bindTrackProfile(int trackId, OpponentProfile profile) {
    profilesByTrack.put(trackId, Objects.requireNonNull(profile, "profile must not be null"));
  }

  /** Returns the profile registered for a team number, if present. */
  public Optional<OpponentProfile> findTeamProfile(int teamNumber) {
    return Optional.ofNullable(profilesByTeam.get(teamNumber));
  }

  /** Returns the profile bound to a live track ID, if present. */
  public Optional<OpponentProfile> resolveForTrack(int trackId) {
    return Optional.ofNullable(profilesByTrack.get(trackId));
  }

  /**
   * Returns the effective defensive threat multiplier for {@code trackId}, or {@code 1.0} if no
   * scouting profile is bound.
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

  /** Returns the number of registered pre-match team profiles. */
  public int registeredTeamCount() {
    return profilesByTeam.size();
  }

  /**
   * Parses a pre-match scouting JSON document using YAECE's zero-dependency {@link MinimalJson}.
   *
   * <p>Expected format:
   *
   * <pre>{@code
   * {
   *   "teams": [
   *     {
   *       "teamNumber": 254,
   *       "role": "PRIMARY_SCORER",
   *       "expectedPointsPerMinute": 38.0,
   *       "preferredSiteThreatMultiplier": 1.2,
   *       "maxSpeedMps": 5.2,
   *       "defenseSusceptibility": 1.15,
   *       "preferredSiteIds": ["alliance_hub_close"]
   *     }
   *   ]
   * }
   * }</pre>
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
      double ppm = MinimalJson.optionalDouble(entry, "expectedPointsPerMinute", 20.0, ctx);
      double siteMult =
          MinimalJson.optionalDouble(entry, "preferredSiteThreatMultiplier", 1.0, ctx);
      double maxSpeed = MinimalJson.optionalDouble(entry, "maxSpeedMps", 4.5, ctx);
      double susceptibility = MinimalJson.optionalDouble(entry, "defenseSusceptibility", 1.0, ctx);
      List<String> sites = MinimalJson.optionalStringList(entry, "preferredSiteIds", ctx);
      registry.registerTeamProfile(
          new OpponentProfile(teamNum, role, ppm, siteMult, maxSpeed, susceptibility, sites));
    }
    return registry;
  }
}
