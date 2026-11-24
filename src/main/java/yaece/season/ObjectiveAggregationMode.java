package yaece.season;

/** How scored pieces accumulate toward a {@link ScoringObjectiveSpec#targetCount()}. */
public enum ObjectiveAggregationMode {
  /** Sum of all matching pieces scored across any matching sites (e.g., 100 Fuel in Hub). */
  TOTAL_COUNT,
  /** Number of distinct matching sites that each have at least 1 matching piece scored. */
  PER_SITE_MINIMUM,
  /** Highest count of matching pieces scored at any single matching site. */
  SAME_SITE_STREAK
}
