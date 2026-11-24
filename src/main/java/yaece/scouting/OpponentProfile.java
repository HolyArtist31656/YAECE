package yaece.scouting;

import java.util.List;
import java.util.Objects;
import yaece.season.MatchPeriod;

/**
 * Immutable pre-match capability profile for a scouted FRC team or tracked opponent robot.
 *
 * @param teamNumber FRC team number (0 when track-only or anonymous)
 * @param rolePrior scouted tactical role archetype
 * @param expectedPointsPerMinute historical or EPA-derived scoring rate (points/min)
 * @param preferredSiteThreatMultiplier additional multiplier applied to base threat (&gt; 0)
 * @param maxSpeedMps estimated top drivetrain speed (m/s)
 * @param defenseSusceptibility how strongly physical defense degrades this robot's cycle rate
 *     (typically 0.5 to 1.8)
 * @param preferredSiteIds optional list of {@code ScoringSiteSpec} IDs this robot favors
 */
public record OpponentProfile(
    int teamNumber,
    RobotRolePrior rolePrior,
    double expectedPointsPerMinute,
    double preferredSiteThreatMultiplier,
    double maxSpeedMps,
    double defenseSusceptibility,
    List<String> preferredSiteIds) {

  private static final double MIN_THREAT_WEIGHT = 0.2;
  private static final double MAX_THREAT_WEIGHT = 4.0;

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
    preferredSiteIds = preferredSiteIds == null ? List.of() : List.copyOf(preferredSiteIds);
  }

  /** Creates a default profile for a team number and role archetype. */
  public static OpponentProfile of(int teamNumber, RobotRolePrior rolePrior) {
    return new OpponentProfile(teamNumber, rolePrior, 20.0, 1.0, 4.5, 1.0, List.of());
  }

  /**
   * Computes the combined defensive threat multiplier for this opponent during {@code period} when
   * approaching {@code inferredSiteId}.
   */
  public double effectiveThreatWeight(
      MatchPeriod period, double matchTimeRemainingSec, String inferredSiteId) {
    double weight =
        rolePrior.baseThreatMultiplier() * preferredSiteThreatMultiplier * defenseSusceptibility;

    // Scale gently by scouted EPA / points-per-minute relative to a 20 pts/min baseline
    if (expectedPointsPerMinute > 0.0) {
      double rateFactor = Math.max(0.6, Math.min(1.8, expectedPointsPerMinute / 20.0));
      weight *= rateFactor;
    }

    if (inferredSiteId != null
        && !inferredSiteId.isEmpty()
        && preferredSiteIds.contains(inferredSiteId)) {
      weight *= 1.35;
    }

    if (period == MatchPeriod.TELEOP
        && matchTimeRemainingSec <= 30.0
        && rolePrior == RobotRolePrior.ENDGAME_SPECIALIST) {
      weight *= 1.5;
    }

    return Math.max(MIN_THREAT_WEIGHT, Math.min(MAX_THREAT_WEIGHT, weight));
  }
}
