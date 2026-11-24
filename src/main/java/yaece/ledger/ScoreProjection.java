package yaece.ledger;

/**
 * Immutable snapshot of reconciled current alliance/opponent scores and extrapolated end-of-match
 * projections.
 *
 * @param currentTimestampSec match elapsed timestamp (seconds) at which this projection was built
 * @param ourCurrentScore reconciled current alliance score (self + teammate + observed scoreboard)
 * @param opponentCurrentScore reconciled current opposing alliance score
 * @param ourScoringRatePps estimated our alliance scoring rate (points/second)
 * @param opponentScoringRatePps extrapolated opposing alliance scoring rate (points/second)
 * @param projectedOurFinalScore projected end-of-match score for our alliance
 * @param projectedOpponentFinalScore projected end-of-match score for the opposing alliance
 * @param opponentProjectionConfidence confidence in [0, 1] based on opponent observation sample
 *     count and recency
 */
public record ScoreProjection(
    double currentTimestampSec,
    double ourCurrentScore,
    double opponentCurrentScore,
    double ourScoringRatePps,
    double opponentScoringRatePps,
    double projectedOurFinalScore,
    double projectedOpponentFinalScore,
    double opponentProjectionConfidence) {

  /** Returns projected final score margin ({@code ourProjected - opponentProjected}). */
  public double projectedMargin() {
    return projectedOurFinalScore - projectedOpponentFinalScore;
  }

  /** Returns current reconciled score margin ({@code ourCurrent - opponentCurrent}). */
  public double currentMargin() {
    return ourCurrentScore - opponentCurrentScore;
  }
}
