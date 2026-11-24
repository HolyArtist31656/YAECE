package yaece.ledger;

/** Runtime toggle controlling whether YAECE optimizes for Match Points or Ranking Points. */
public enum OptimizationObjective {
  /** Maximize expected match points per second (including bonus match-point objectives). */
  MATCH_POINTS,
  /**
   * Heavily prioritize unfinished, reachable Ranking Point objectives; automatically spills over to
   * {@link #MATCH_POINTS} once all reachable RPs are secured.
   */
  RANKING_POINTS
}
