// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.scouting;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import yaece.json.MinimalJson;

/**
 * Immutable 6-robot FRC match roster containing our own team number, up to 2 alliance partners,
 * and up to 3 opposing teams. Used by {@link ScoutingPriorRegistry} and {@code DefenseInterceptor}
 * to resolve per-robot capabilities and fall back cleanly to {@link
 * OpponentProfile#unknownDefaults(int)} for un-scouted teams.
 *
 * @param ownTeamNumber Our FRC team number (&gt; 0).
 * @param allyTeamNumbers Up to 2 alliance partner team numbers (distinct, disjoint from own and
 *     opponents).
 * @param opponentTeamNumbers Up to 3 opposing alliance team numbers (distinct, disjoint from own
 *     and allies).
 */
public record MatchRoster(
    int ownTeamNumber, List<Integer> allyTeamNumbers, List<Integer> opponentTeamNumbers) {

  /** Maximum number of alliance partner robots in a standard 3v3 FRC match. */
  public static final int MAX_ALLIES = 2;

  /** Maximum number of opposing robots in a standard 3v3 FRC match. */
  public static final int MAX_OPPONENTS = 3;

  /** Compact constructor validating roster integrity and uniqueness. */
  public MatchRoster {
    if (ownTeamNumber <= 0) {
      throw new IllegalArgumentException("ownTeamNumber must be > 0");
    }
    allyTeamNumbers = validateTeamList(allyTeamNumbers, "allyTeamNumbers", MAX_ALLIES);
    opponentTeamNumbers =
        validateTeamList(opponentTeamNumbers, "opponentTeamNumbers", MAX_OPPONENTS);

    if (allyTeamNumbers.contains(ownTeamNumber)) {
      throw new IllegalArgumentException(
          "allyTeamNumbers cannot include ownTeamNumber (" + ownTeamNumber + ")");
    }
    if (opponentTeamNumbers.contains(ownTeamNumber)) {
      throw new IllegalArgumentException(
          "opponentTeamNumbers cannot include ownTeamNumber (" + ownTeamNumber + ")");
    }
    for (int ally : allyTeamNumbers) {
      if (opponentTeamNumbers.contains(ally)) {
        throw new IllegalArgumentException(
            "Team " + ally + " cannot appear in both allyTeamNumbers and opponentTeamNumbers");
      }
    }
  }

  /**
   * Creates a {@link MatchRoster} from our team number, ally team numbers, and opponent team
   * numbers.
   *
   * @param ownTeamNumber Our FRC team number (&gt; 0).
   * @param allyTeamNumbers Up to 2 alliance partner team numbers.
   * @param opponentTeamNumbers Up to 3 opposing alliance team numbers.
   * @return Validated {@link MatchRoster}.
   */
  public static MatchRoster of(
      int ownTeamNumber, List<Integer> allyTeamNumbers, List<Integer> opponentTeamNumbers) {
    return new MatchRoster(ownTeamNumber, allyTeamNumbers, opponentTeamNumbers);
  }

  /**
   * Returns true if {@code teamNumber} is our own team.
   *
   * @param teamNumber Team number to test.
   * @return True when {@code teamNumber == ownTeamNumber}.
   */
  public boolean isOwnTeam(int teamNumber) {
    return teamNumber == ownTeamNumber;
  }

  /**
   * Returns true if {@code teamNumber} is listed as an alliance partner.
   *
   * @param teamNumber Team number to test.
   * @return True when {@code teamNumber} is an ally.
   */
  public boolean isAlly(int teamNumber) {
    return allyTeamNumbers.contains(teamNumber);
  }

  /**
   * Returns true if {@code teamNumber} is listed as an opponent.
   *
   * @param teamNumber Team number to test.
   * @return True when {@code teamNumber} is an opponent.
   */
  public boolean isOpponent(int teamNumber) {
    return opponentTeamNumbers.contains(teamNumber);
  }

  /**
   * Returns all team numbers in the match (own team, then allies, then opponents).
   *
   * @return Unmodifiable list of all team numbers in the roster.
   */
  public List<Integer> allTeams() {
    List<Integer> all = new ArrayList<>(1 + allyTeamNumbers.size() + opponentTeamNumbers.size());
    all.add(ownTeamNumber);
    all.addAll(allyTeamNumbers);
    all.addAll(opponentTeamNumbers);
    return List.copyOf(all);
  }

  /**
   * Resolves {@link OpponentProfile} entries for every ally in this roster using {@code registry},
   * falling back to {@link OpponentProfile#unknownDefaults(int)} for un-scouted teams.
   *
   * @param registry Scouting prior registry.
   * @return Unmodifiable insertion-ordered map from ally team number to resolved profile.
   */
  public Map<Integer, OpponentProfile> resolveAllyProfiles(ScoutingPriorRegistry registry) {
    Objects.requireNonNull(registry, "registry must not be null");
    return resolveProfilesFor(allyTeamNumbers, registry);
  }

  /**
   * Resolves {@link OpponentProfile} entries for every opponent in this roster using {@code
   * registry}, falling back to {@link OpponentProfile#unknownDefaults(int)} for un-scouted teams.
   *
   * @param registry Scouting prior registry.
   * @return Unmodifiable insertion-ordered map from opponent team number to resolved profile.
   */
  public Map<Integer, OpponentProfile> resolveOpponentProfiles(ScoutingPriorRegistry registry) {
    Objects.requireNonNull(registry, "registry must not be null");
    return resolveProfilesFor(opponentTeamNumbers, registry);
  }

  /**
   * Parses a {@link MatchRoster} from JSON using YAECE's zero-dependency {@link MinimalJson}.
   *
   * <p>Expected format:
   *
   * <pre>{@code
   * {
   *   "ownTeam": 2181,
   *   "allies": [254, 1678],
   *   "opponents": [6328, 2910, 3847]
   * }
   * }</pre>
   *
   * @param json Raw JSON object text.
   * @return Parsed {@link MatchRoster}.
   */
  public static MatchRoster fromJson(String json) {
    Objects.requireNonNull(json, "json must not be null");
    return fromJsonMap(MinimalJson.parseObject(json), "matchRoster");
  }

  /** Package-private parser shared with {@link ScoutingPriorRegistry#fromJson(String)}. */
  static MatchRoster fromJsonMap(Map<String, Object> obj, String ctx) {
    int ownTeam = MinimalJson.requireInt(obj, "ownTeam", ctx);
    List<Integer> allies = parseIntList(obj, "allies", ctx);
    List<Integer> opponents = parseIntList(obj, "opponents", ctx);
    return new MatchRoster(ownTeam, allies, opponents);
  }

  private static List<Integer> parseIntList(Map<String, Object> obj, String key, String ctx) {
    List<Object> raw = MinimalJson.optionalArray(obj, key, ctx);
    List<Integer> out = new ArrayList<>(raw.size());
    for (int i = 0; i < raw.size(); i++) {
      Object item = raw.get(i);
      if (!(item instanceof Number num)) {
        throw new IllegalArgumentException(ctx + "." + key + "[" + i + "] must be an integer");
      }
      out.add(num.intValue());
    }
    return out;
  }

  private static Map<Integer, OpponentProfile> resolveProfilesFor(
      List<Integer> teams, ScoutingPriorRegistry registry) {
    Map<Integer, OpponentProfile> out = new LinkedHashMap<>(teams.size());
    for (int team : teams) {
      out.put(team, registry.findTeamProfile(team).orElseGet(() -> OpponentProfile.unknownDefaults(team)));
    }
    return Collections.unmodifiableMap(out);
  }

  private static List<Integer> validateTeamList(List<Integer> raw, String name, int maxSize) {
    if (raw == null || raw.isEmpty()) {
      return List.of();
    }
    if (raw.size() > maxSize) {
      throw new IllegalArgumentException(
          name + " cannot contain more than " + maxSize + " teams (got " + raw.size() + ")");
    }
    Set<Integer> seen = new LinkedHashSet<>(raw.size());
    for (Integer team : raw) {
      if (team == null || team <= 0) {
        throw new IllegalArgumentException(name + " entries must be positive team numbers");
      }
      if (!seen.add(team)) {
        throw new IllegalArgumentException(name + " contains duplicate team number " + team);
      }
    }
    return List.copyOf(seen);
  }
}
