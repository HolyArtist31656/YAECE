package yaece.season;

import java.util.Objects;
import java.util.Set;

/**
 * Optional, game-agnostic clock-based scoring interval defined in a season JSON file (e.g., Endgame
 * double-point windows, timed bonus periods, or fixed shift schedules).
 *
 * <p>Seasons without fixed timed multipliers simply omit {@code scoringIntervals} from their JSON.
 *
 * @param id unique interval identifier
 * @param startElapsedSec match elapsed start time in seconds (inclusive, &gt;= 0)
 * @param endElapsedSec match elapsed end time in seconds (exclusive, &gt; startElapsedSec)
 * @param matchingSiteIds optional set of site IDs affected (empty + empty tags = all sites)
 * @param matchingSiteTags optional set of site tags affected
 * @param active whether matching sites are active (scorable) during this interval
 * @param pointMultiplier multiplicative factor applied to base site points (&gt;= 0)
 * @param flatBonusPoints flat additive points per scored piece during this interval
 */
public record ScoringIntervalSpec(
    String id,
    double startElapsedSec,
    double endElapsedSec,
    Set<String> matchingSiteIds,
    Set<String> matchingSiteTags,
    boolean active,
    double pointMultiplier,
    double flatBonusPoints) {

  public ScoringIntervalSpec {
    if (id == null || id.isBlank()) {
      throw new IllegalArgumentException("ScoringIntervalSpec.id must be non-blank");
    }
    if (!Double.isFinite(startElapsedSec) || startElapsedSec < 0.0) {
      throw new IllegalArgumentException("startElapsedSec must be finite and >= 0");
    }
    if (!Double.isFinite(endElapsedSec) || endElapsedSec <= startElapsedSec) {
      throw new IllegalArgumentException("endElapsedSec must be finite and > startElapsedSec");
    }
    matchingSiteIds = matchingSiteIds == null ? Set.of() : Set.copyOf(matchingSiteIds);
    matchingSiteTags = matchingSiteTags == null ? Set.of() : Set.copyOf(matchingSiteTags);
    if (!Double.isFinite(pointMultiplier) || pointMultiplier < 0.0) {
      throw new IllegalArgumentException("pointMultiplier must be finite and >= 0");
    }
    if (!Double.isFinite(flatBonusPoints)) {
      throw new IllegalArgumentException("flatBonusPoints must be finite");
    }
  }

  /**
   * Returns true if {@code elapsedMatchSec} falls within {@code [startElapsedSec, endElapsedSec)}.
   */
  public boolean isActiveAt(double elapsedMatchSec) {
    return elapsedMatchSec >= startElapsedSec && elapsedMatchSec < endElapsedSec;
  }

  /** Returns true if this interval applies to {@code site}. */
  public boolean matchesSite(ScoringSiteSpec site) {
    Objects.requireNonNull(site, "site must not be null");
    boolean idFilter = !matchingSiteIds.isEmpty();
    boolean tagFilter = !matchingSiteTags.isEmpty();
    if (!idFilter && !tagFilter) {
      return true;
    }
    if (idFilter && matchingSiteIds.contains(site.id())) {
      return true;
    }
    if (tagFilter) {
      for (String tag : matchingSiteTags) {
        if (site.hasTag(tag)) {
          return true;
        }
      }
    }
    return false;
  }
}
