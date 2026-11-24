package yaece.ledger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import yaece.season.MatchPeriod;
import yaece.season.ScoringObjectiveSpec;
import yaece.season.ScoringSiteSpec;
import yaece.season.SeasonConfig;
import yaece.tactics.DynamicScoringState;

/**
 * Flexible, timestamp-aware self + alliance + opposing-alliance scoring ledger and opponent score
 * extrapolator.
 *
 * <p>Key reconciliation guarantees:
 *
 * <ul>
 *   <li>Every self-scored piece is recorded with a match-elapsed timestamp so human Driver Station
 *       scoreboard observations never double-count pieces scored before the observation timestamp,
 *       while automatically adding new self-scored pieces recorded <em>after</em> that timestamp.
 *   <li>Optional human estimates of teammate contributions, total alliance scored pieces, and
 *       opposing-alliance score/pieces are smoothly fused when present and gracefully omitted when
 *       absent.
 *   <li>Timestamped opposing-alliance observations feed an exponentially-weighted linear regression
 *       extrapolator to forecast end-of-match score margins.
 * </ul>
 */
public final class MatchScoreLedger {

  private record SelfScoreEvent(
      String siteId, String pieceId, MatchPeriod period, double points, double timestampSec) {}

  private record TimedScalar(double value, double timestampSec) {}

  private record TimedCount(int count, double timestampSec) {}

  private static final double EXP_DECAY_TAU_SEC = 25.0;
  private static final double MAX_EXTRAPOLATED_PPS = 15.0;

  private final SeasonConfig season;
  private final List<SelfScoreEvent> selfEvents = new ArrayList<>();

  // Optional human Driver Station console inputs for our alliance
  private TimedScalar observedAllianceScore = null;
  private TimedScalar teammateEstimatedPoints = null;
  private final Map<String, TimedCount> teammateEstimatedSiteCounts = new HashMap<>();
  private final Map<String, TimedCount> observedAllianceObjectiveCounts = new HashMap<>();

  // Optional human Driver Station console inputs for opposing alliance
  private final List<TimedScalar> opponentScoreHistory = new ArrayList<>();
  private final Map<String, TimedCount> opponentPieceCounts = new HashMap<>();
  private final List<TimedScalar> opponentPieceImpliedHistory = new ArrayList<>();

  /** Creates a fresh match ledger bound to the given {@link SeasonConfig}. */
  public MatchScoreLedger(SeasonConfig season) {
    this.season = Objects.requireNonNull(season, "season must not be null");
  }

  /** Clears all match events and console inputs (call at start of each match). */
  public void reset() {
    selfEvents.clear();
    observedAllianceScore = null;
    teammateEstimatedPoints = null;
    teammateEstimatedSiteCounts.clear();
    observedAllianceObjectiveCounts.clear();
    opponentScoreHistory.clear();
    opponentPieceCounts.clear();
    opponentPieceImpliedHistory.clear();
  }

  // -------------------------------------------------------------------------
  // 1. Self Robot Tracked Scoring Events
  // -------------------------------------------------------------------------

  /**
   * Records a game piece scored by our own robot at a specific site and match-elapsed timestamp.
   *
   * @param siteId valid {@link ScoringSiteSpec#id()} in the loaded season config
   * @param pieceId valid game-piece ID accepted by that site
   * @param period {@link MatchPeriod#AUTO} or {@link MatchPeriod#TELEOP}
   * @param elapsedMatchSec elapsed match time in seconds (&gt;= 0)
   */
  public void recordSelfScored(
      String siteId, String pieceId, MatchPeriod period, double elapsedMatchSec) {
    recordSelfScored(siteId, pieceId, period, elapsedMatchSec, null);
  }

  /**
   * Records a game piece scored by our own robot, applying any active {@link DynamicScoringState}
   * or {@link yaece.season.ScoringIntervalSpec} multipliers and flat bonuses.
   */
  public void recordSelfScored(
      String siteId,
      String pieceId,
      MatchPeriod period,
      double elapsedMatchSec,
      DynamicScoringState dynamicScoring) {
    ScoringSiteSpec site =
        season
            .findScoringSite(siteId)
            .orElseThrow(() -> new IllegalArgumentException("Unknown scoringSite: " + siteId));
    if (!site.acceptsPiece(pieceId)) {
      throw new IllegalArgumentException(
          "Site '" + siteId + "' does not accept piece '" + pieceId + "'");
    }
    validateTimestamp(elapsedMatchSec);
    double pts =
        dynamicScoring != null
            ? dynamicScoring
                .evaluateSite(season, site, period, elapsedMatchSec)
                .effectiveBasePoints()
            : site.pointsFor(period);
    selfEvents.add(new SelfScoreEvent(siteId, pieceId, period, pts, elapsedMatchSec));
  }

  /** Returns the exact count of pieces scored by our own robot at {@code siteId}. */
  public int getSelfScoredCountAtSite(String siteId) {
    int count = 0;
    for (SelfScoreEvent e : selfEvents) {
      if (e.siteId().equals(siteId)) {
        count++;
      }
    }
    return count;
  }

  /** Returns the total points scored solely by our own robot. */
  public double getSelfScoredPoints() {
    double sum = 0.0;
    for (SelfScoreEvent e : selfEvents) {
      sum += e.points();
    }
    return sum;
  }

  // -------------------------------------------------------------------------
  // 2. Optional Human Driver Station Console Inputs (Our Alliance)
  // -------------------------------------------------------------------------

  /**
   * Records a human-observed alliance scoreboard score at {@code timestampSec}.
   *
   * <p>Self-scored events with {@code t <= timestampSec} are treated as already reflected in the
   * observed scoreboard; self-scored events with {@code t > timestampSec} are added on top.
   */
  public void updateObservedAllianceScore(double observedScore, double timestampSec) {
    validateNonNegativeFinite(observedScore, "observedScore");
    validateTimestamp(timestampSec);
    if (observedAllianceScore == null || timestampSec >= observedAllianceScore.timestampSec()) {
      observedAllianceScore = new TimedScalar(observedScore, timestampSec);
    }
  }

  /**
   * Records a human estimate of cumulative teammate-only points scored up to {@code timestampSec}.
   */
  public void updateTeammateEstimatedPoints(double teammatePoints, double timestampSec) {
    validateNonNegativeFinite(teammatePoints, "teammatePoints");
    validateTimestamp(timestampSec);
    if (teammateEstimatedPoints == null || timestampSec >= teammateEstimatedPoints.timestampSec()) {
      teammateEstimatedPoints = new TimedScalar(teammatePoints, timestampSec);
    }
  }

  /** Records a human estimate of cumulative teammate pieces scored at a specific site. */
  public void updateTeammateSiteCount(String siteId, int teammateCount, double timestampSec) {
    if (season.findScoringSite(siteId).isEmpty()) {
      throw new IllegalArgumentException("Unknown scoringSite: " + siteId);
    }
    if (teammateCount < 0) {
      throw new IllegalArgumentException("teammateCount must be >= 0");
    }
    validateTimestamp(timestampSec);
    teammateEstimatedSiteCounts.put(siteId, new TimedCount(teammateCount, timestampSec));
  }

  /**
   * Records a human-observed total alliance piece count toward a specific {@link
   * ScoringObjectiveSpec#id()} at {@code timestampSec}.
   */
  public void updateObservedAllianceObjectiveCount(
      String objectiveId, int observedTotalCount, double timestampSec) {
    if (season.findScoringObjective(objectiveId).isEmpty()) {
      throw new IllegalArgumentException("Unknown scoringObjective: " + objectiveId);
    }
    if (observedTotalCount < 0) {
      throw new IllegalArgumentException("observedTotalCount must be >= 0");
    }
    validateTimestamp(timestampSec);
    observedAllianceObjectiveCounts.put(
        objectiveId, new TimedCount(observedTotalCount, timestampSec));
  }

  // -------------------------------------------------------------------------
  // 3. Optional Human Inputs for Opposing Alliance
  // -------------------------------------------------------------------------

  /** Records a timestamped human-observed opposing alliance scoreboard score. */
  public void updateOpponentObservedScore(double observedScore, double timestampSec) {
    validateNonNegativeFinite(observedScore, "observedScore");
    validateTimestamp(timestampSec);
    insertSorted(opponentScoreHistory, new TimedScalar(observedScore, timestampSec));
  }

  /**
   * Records a timestamped human estimate of opposing alliance cumulative scored pieces for a given
   * game-piece type.
   */
  public void updateOpponentObservedPieces(
      String pieceId, int estimatedTotalCount, double timestampSec) {
    if (season.findGamePiece(pieceId).isEmpty()) {
      throw new IllegalArgumentException("Unknown gamePiece: " + pieceId);
    }
    if (estimatedTotalCount < 0) {
      throw new IllegalArgumentException("estimatedTotalCount must be >= 0");
    }
    validateTimestamp(timestampSec);
    opponentPieceCounts.put(pieceId, new TimedCount(estimatedTotalCount, timestampSec));
    double impliedTotal = computeOpponentImpliedPiecePoints();
    insertSorted(opponentPieceImpliedHistory, new TimedScalar(impliedTotal, timestampSec));
  }

  // -------------------------------------------------------------------------
  // 4. Reconciled Queries & Objective Progress
  // -------------------------------------------------------------------------

  /**
   * Returns the reconciled estimated total score for our alliance, avoiding double-counting between
   * self-tracked events, teammate estimates, and timestamped scoreboard observations.
   */
  public double getEstimatedAllianceScore() {
    double selfTotal = getSelfScoredPoints();
    double teammatePts = estimateTeammatePoints();

    if (observedAllianceScore == null) {
      return selfTotal + teammatePts;
    }

    double tObs = observedAllianceScore.timestampSec();
    double selfAtObs = 0.0;
    double selfAfterObs = 0.0;
    for (SelfScoreEvent e : selfEvents) {
      if (e.timestampSec() <= tObs) {
        selfAtObs += e.points();
      } else {
        selfAfterObs += e.points();
      }
    }

    double teammateDeltaSinceObs = 0.0;
    if (teammateEstimatedPoints != null && teammateEstimatedPoints.timestampSec() > tObs) {
      double impliedTeammateAtObs = Math.max(0.0, observedAllianceScore.value() - selfAtObs);
      teammateDeltaSinceObs = Math.max(0.0, teammateEstimatedPoints.value() - impliedTeammateAtObs);
    }

    double baseAtObs = Math.max(observedAllianceScore.value(), selfAtObs + teammatePts);
    return baseAtObs + selfAfterObs + teammateDeltaSinceObs;
  }

  /** Returns reconciled total piece count (self + optional teammate estimate) at {@code siteId}. */
  public int getEstimatedSiteCount(String siteId) {
    int selfCount = getSelfScoredCountAtSite(siteId);
    TimedCount tm = teammateEstimatedSiteCounts.get(siteId);
    return selfCount + (tm != null ? tm.count() : 0);
  }

  /**
   * Returns reconciled alliance progress toward a {@link ScoringObjectiveSpec}, combining
   * self-tracked events, teammate site counts, and any timestamped console total-count override
   * plus self pieces scored after that timestamp.
   */
  public int getEstimatedObjectiveProgress(String objectiveId) {
    ScoringObjectiveSpec obj =
        season
            .findScoringObjective(objectiveId)
            .orElseThrow(
                () -> new IllegalArgumentException("Unknown scoringObjective: " + objectiveId));

    int computedProgress = computeObjectiveProgressFromEventsAndTeammates(obj);
    TimedCount observed = observedAllianceObjectiveCounts.get(objectiveId);
    if (observed == null) {
      return computedProgress;
    }

    int selfAfterObs = 0;
    for (SelfScoreEvent e : selfEvents) {
      if (e.timestampSec() > observed.timestampSec()) {
        ScoringSiteSpec site = season.findScoringSite(e.siteId()).orElse(null);
        if (site != null && obj.matches(site, e.pieceId())) {
          selfAfterObs++;
        }
      }
    }
    return Math.max(computedProgress, observed.count() + selfAfterObs);
  }

  /** Returns whether the given objective has reached its target count. */
  public boolean isObjectiveComplete(String objectiveId) {
    ScoringObjectiveSpec obj =
        season
            .findScoringObjective(objectiveId)
            .orElseThrow(
                () -> new IllegalArgumentException("Unknown scoringObjective: " + objectiveId));
    return getEstimatedObjectiveProgress(objectiveId) >= obj.targetCount();
  }

  /**
   * Extrapolates both alliances' scores to the end of the match using timestamped observations and
   * exponentially-weighted linear regression on the opposing alliance's scoring trend.
   */
  public ScoreProjection projectScores(double currentTimestampSec) {
    validateTimestamp(currentTimestampSec);
    double remainingSec = Math.max(0.0, season.matchDurationSeconds() - currentTimestampSec);
    double ourCurrent = getEstimatedAllianceScore();
    double ourRate = currentTimestampSec > 1.0 ? ourCurrent / currentTimestampSec : 0.0;
    double ourProjectedFinal = ourCurrent + ourRate * remainingSec;

    List<TimedScalar> oppSeries = selectPrimaryOpponentSeries();
    if (oppSeries.isEmpty()) {
      return new ScoreProjection(
          currentTimestampSec, ourCurrent, 0.0, ourRate, 0.0, ourProjectedFinal, 0.0, 0.0);
    }

    TimedScalar latest = oppSeries.get(oppSeries.size() - 1);
    double oppRate = estimateOpponentRatePps(oppSeries, currentTimestampSec);
    double dtSinceLatest = Math.max(0.0, currentTimestampSec - latest.timestampSec());
    double oppCurrent = latest.value() + oppRate * dtSinceLatest;
    double oppProjectedFinal = oppCurrent + oppRate * remainingSec;

    double sampleFactor = Math.min(1.0, oppSeries.size() / 3.0);
    double freshnessFactor = Math.exp(-dtSinceLatest / 30.0);
    double confidence = Math.max(0.0, Math.min(1.0, sampleFactor * freshnessFactor));

    return new ScoreProjection(
        currentTimestampSec,
        ourCurrent,
        oppCurrent,
        ourRate,
        oppRate,
        ourProjectedFinal,
        oppProjectedFinal,
        confidence);
  }

  // -------------------------------------------------------------------------
  // Internal Helpers
  // -------------------------------------------------------------------------

  private double estimateTeammatePoints() {
    double fromSites = 0.0;
    for (Map.Entry<String, TimedCount> entry : teammateEstimatedSiteCounts.entrySet()) {
      ScoringSiteSpec site = season.findScoringSite(entry.getKey()).orElse(null);
      if (site != null) {
        fromSites += entry.getValue().count() * site.teleopPoints();
      }
    }
    double direct = teammateEstimatedPoints != null ? teammateEstimatedPoints.value() : 0.0;
    return Math.max(direct, fromSites);
  }

  private int computeObjectiveProgressFromEventsAndTeammates(ScoringObjectiveSpec obj) {
    Map<String, Integer> perSite = new HashMap<>();
    int total = 0;
    for (SelfScoreEvent e : selfEvents) {
      ScoringSiteSpec site = season.findScoringSite(e.siteId()).orElse(null);
      if (site != null && obj.matches(site, e.pieceId())) {
        total++;
        perSite.merge(site.id(), 1, Integer::sum);
      }
    }
    for (Map.Entry<String, TimedCount> entry : teammateEstimatedSiteCounts.entrySet()) {
      ScoringSiteSpec site = season.findScoringSite(entry.getKey()).orElse(null);
      if (site != null) {
        boolean siteMatches = false;
        for (String pId : site.acceptedPieceIds()) {
          if (obj.matches(site, pId)) {
            siteMatches = true;
            break;
          }
        }
        if (siteMatches && entry.getValue().count() > 0) {
          total += entry.getValue().count();
          perSite.merge(site.id(), entry.getValue().count(), Integer::sum);
        }
      }
    }
    return switch (obj.aggregationMode()) {
      case TOTAL_COUNT -> total;
      case PER_SITE_MINIMUM -> perSite.size();
      case SAME_SITE_STREAK -> {
        int maxStreak = 0;
        for (int c : perSite.values()) {
          if (c > maxStreak) {
            maxStreak = c;
          }
        }
        yield maxStreak;
      }
    };
  }

  private double computeOpponentImpliedPiecePoints() {
    double sum = 0.0;
    for (Map.Entry<String, TimedCount> entry : opponentPieceCounts.entrySet()) {
      sum += entry.getValue().count() * season.nominalTeleopPointsForPiece(entry.getKey());
    }
    return sum;
  }

  private List<TimedScalar> selectPrimaryOpponentSeries() {
    if (!opponentScoreHistory.isEmpty()) {
      return opponentScoreHistory;
    }
    return opponentPieceImpliedHistory;
  }

  private static double estimateOpponentRatePps(
      List<TimedScalar> series, double currentTimestampSec) {
    if (series.size() == 1) {
      TimedScalar only = series.get(0);
      if (only.timestampSec() <= 1.0) {
        return 0.0;
      }
      return clampRate(only.value() / only.timestampSec());
    }

    // Exponentially-weighted least squares slope d(score)/dt
    double sumW = 0.0;
    double sumWT = 0.0;
    double sumWY = 0.0;
    for (TimedScalar pt : series) {
      double age = Math.max(0.0, currentTimestampSec - pt.timestampSec());
      double w = Math.exp(-age / EXP_DECAY_TAU_SEC);
      sumW += w;
      sumWT += w * pt.timestampSec();
      sumWY += w * pt.value();
    }
    if (sumW <= 1e-9) {
      return 0.0;
    }
    double meanT = sumWT / sumW;
    double meanY = sumWY / sumW;

    double num = 0.0;
    double den = 0.0;
    for (TimedScalar pt : series) {
      double age = Math.max(0.0, currentTimestampSec - pt.timestampSec());
      double w = Math.exp(-age / EXP_DECAY_TAU_SEC);
      double dt = pt.timestampSec() - meanT;
      num += w * dt * (pt.value() - meanY);
      den += w * dt * dt;
    }
    if (den <= 1e-9) {
      return 0.0;
    }
    return clampRate(num / den);
  }

  private static double clampRate(double rawPps) {
    if (!Double.isFinite(rawPps) || rawPps < 0.0) {
      return 0.0;
    }
    return Math.min(rawPps, MAX_EXTRAPOLATED_PPS);
  }

  private static void insertSorted(List<TimedScalar> list, TimedScalar sample) {
    int idx = list.size();
    while (idx > 0 && list.get(idx - 1).timestampSec() > sample.timestampSec()) {
      idx--;
    }
    if (idx > 0 && Math.abs(list.get(idx - 1).timestampSec() - sample.timestampSec()) < 1e-6) {
      list.set(idx - 1, sample);
    } else {
      list.add(idx, sample);
    }
  }

  private static void validateTimestamp(double timestampSec) {
    if (!Double.isFinite(timestampSec) || timestampSec < 0.0) {
      throw new IllegalArgumentException("timestampSec must be finite and >= 0");
    }
  }

  private static void validateNonNegativeFinite(double value, String name) {
    if (!Double.isFinite(value) || value < 0.0) {
      throw new IllegalArgumentException(name + " must be finite and >= 0");
    }
  }
}
