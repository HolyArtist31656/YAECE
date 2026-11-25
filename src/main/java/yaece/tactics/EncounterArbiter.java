// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.console.PlayMode;
import yaece.ledger.MatchScoreLedger;
import yaece.ledger.OptimizationObjective;
import yaece.ledger.ScoreProjection;
import yaece.scouting.ScoutingPriorRegistry;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.GamePieceSpec;
import yaece.season.MatchPeriod;
import yaece.season.ObjectiveRewardType;
import yaece.season.ScoringObjectiveSpec;
import yaece.season.ScoringSiteSpec;
import yaece.season.SeasonConfig;
import yaece.world.TrackedGamePiece;
import yaece.world.TrackedRobot;
import yaece.world.WorldSnapshot;

/**
 * 20–50 Hz receding-horizon tactical arbiter deciding between {@link EncounterIntent#SCORE}, {@link
 * EncounterIntent#GATHER}, {@link EncounterIntent#HOLD_STAGE}, and on-the-fly {@link
 * PlayMode#DEFENSE}.
 *
 * <p>Optimizes marginal Points-Per-Second (PPS) with linked bonus/RP completion-urgency curves,
 * remaining-time reachability gating, arrival-time race checks ("Bot Aborter"), 6-DoF intake
 * orientation alignment, and anti-thrashing hysteresis.
 */
public final class EncounterArbiter {
  private static final double OPPONENT_RACE_ABORT_MARGIN_SEC = 0.65;
  private static final double HYSTERESIS_BONUS_RATIO = 0.15;
  private static final double MIN_COMMIT_DWELL_SEC = 0.35;
  private static final double RP_PRIORITY_WEIGHT = 12.0;
  private static final double PRE_OPEN_STAGING_WINDOW_SEC = 3.5;
  private static final DynamicScoringState DEFAULT_DYNAMIC = new DynamicScoringState();

  private static final double ALLY_DECONFLICT_MARGIN_SEC = 0.25;

  private final SeasonConfig season;
  private final RobotKinematicsSpec kinematics;
  private final DefenseInterceptor defenseInterceptor;
  private final FieldPathfinder pathfinder;

  private EncounterIntent lastIntent = EncounterIntent.IDLE;
  private String lastTargetId = "";
  private double lastSwitchTimestampSec = -10.0;

  public EncounterArbiter(SeasonConfig season, RobotKinematicsSpec kinematics) {
    this(season, kinematics, new ScoutingPriorRegistry());
  }

  public EncounterArbiter(
      SeasonConfig season, RobotKinematicsSpec kinematics, ScoutingPriorRegistry scoutingPriors) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.kinematics = Objects.requireNonNull(kinematics, "kinematics must not be null");
    this.defenseInterceptor =
        new DefenseInterceptor(
            season,
            kinematics,
            Objects.requireNonNull(scoutingPriors, "scoutingPriors must not be null"));
    this.pathfinder = new FieldPathfinder(season, kinematics.bumperRadiusMeters());
  }

  /** Evaluates the optimal tactical recommendation for the current tick. */
  public TacticalRecommendation evaluate(
      WorldSnapshot world,
      MatchScoreLedger ledger,
      RobotInventoryState inventory,
      AllianceSide ourAlliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      PlayMode playMode,
      DefenseStrategyMode defenseStrategyMode,
      OptimizationObjective optimizationObjective) {
    return evaluate(
        world,
        ledger,
        inventory,
        ourAlliance,
        period,
        matchTimeRemainingSec,
        playMode,
        defenseStrategyMode,
        optimizationObjective,
        DEFAULT_DYNAMIC);
  }

  public TacticalRecommendation evaluate(
      WorldSnapshot world,
      MatchScoreLedger ledger,
      RobotInventoryState inventory,
      AllianceSide ourAlliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      PlayMode playMode,
      DefenseStrategyMode defenseStrategyMode,
      OptimizationObjective optimizationObjective,
      DynamicScoringState dynamicScoring) {
    Objects.requireNonNull(world, "world must not be null");
    Objects.requireNonNull(ledger, "ledger must not be null");
    Objects.requireNonNull(inventory, "inventory must not be null");

    double nowSec = world.timestampSec();
    double elapsedMatchSec = Math.max(0.0, season.matchDurationSeconds() - matchTimeRemainingSec);
    DynamicScoringState dyn = dynamicScoring != null ? dynamicScoring : DEFAULT_DYNAMIC;

    if (playMode == PlayMode.DEFENSE) {
      ScoreProjection proj =
          ledger.projectScores(
              Math.max(0.0, season.matchDurationSeconds() - matchTimeRemainingSec));
      TacticalRecommendation defRec =
          defenseInterceptor.evaluate(
              world, ourAlliance, period, matchTimeRemainingSec, defenseStrategyMode, proj);
      recordCommitted(defRec, nowSec);
      return defRec;
    }

    Translation2d egoXy = world.egoPose3d().getTranslation().toTranslation2d();
    TacticalRecommendation best = null;
    double bestEffectiveUtility = -1.0;

    // 1. Evaluate SCORE (and HOLD_STAGE fallback if lane blocked) for every held piece
    if (inventory.hasAnyHeld() && inventory.mechanismReadyToScore()) {
      for (ScoringSiteSpec site : season.scoringSites()) {
        if (!site.isUnlimitedCapacity()
            && ledger.getEstimatedSiteCount(site.id()) >= site.siteCapacity()) {
          continue;
        }
        for (String heldPieceId : site.acceptedPieceIds()) {
          if (inventory.countOf(heldPieceId) <= 0) {
            continue;
          }
          Pose3d approachPose = season.resolveSiteApproachPose(site, ourAlliance);
          Translation2d approachXy = approachPose.getTranslation().toTranslation2d();
          FieldPathfinder.PathResult scorePath =
              pathfinder.findPath(egoXy, approachXy, ourAlliance, period, matchTimeRemainingSec);
          double tDrive = kinematics.estimateTravelTimeSec(scorePath.totalDistanceMeters());
          Pose3d scoreWaypointPose =
              scorePath.detourRequired()
                  ? new Pose3d(
                      new Translation3d(
                          scorePath.nextWaypoint().getX(), scorePath.nextWaypoint().getY(), 0.0),
                      approachPose.getRotation())
                  : approachPose;
          double lanePenalty =
              LaneClearanceChecker.evaluateLaneCongestionPenaltySec(
                  egoXy, approachXy, tDrive, world, kinematics.bumperRadiusMeters());

          DynamicScoringState.EffectiveSiteScoring eff =
              dyn.evaluateSite(season, site, period, elapsedMatchSec);
          double arrivalEst = tDrive + kinematics.nominalScoreCycleSec();

          // Inactive site OR closing window before we can arrive: skip scoring, pre-stage if
          // opening soon or hopper full
          if (!eff.active() || eff.activeWindowRemainingSec() < arrivalEst) {
            boolean openingSoon =
                !eff.active() && eff.inactiveTimeUntilOpenSec() <= PRE_OPEN_STAGING_WINDOW_SEC;
            boolean hopperFull =
                !eff.active() && inventory.totalHeld() >= season.maxTotalHeldGamePieces();
            if (openingSoon || hopperFull) {
              Pose2d hold2d = season.resolveSiteStagingPose(site, ourAlliance);
              double tHold =
                  kinematics.estimateTravelTimeSec(egoXy.getDistance(hold2d.getTranslation()));
              Pose3d hold3d =
                  new Pose3d(
                      new Translation3d(hold2d.getX(), hold2d.getY(), 0.0),
                      new Rotation3d(0.0, 0.0, hold2d.getRotation().getRadians()));
              double stageUtil = openingSoon ? 0.55 : 0.12;
              String stageReason =
                  openingSoon
                      ? "Pre-staging for " + site.id() + " opening soon"
                      : "Hopper full while " + site.id() + " inactive; pre-staging at hold pose";
              TacticalRecommendation preStage =
                  new TacticalRecommendation(
                      EncounterIntent.HOLD_STAGE,
                      hold3d,
                      site.id(),
                      heldPieceId,
                      stageUtil,
                      tHold,
                      false,
                      null,
                      stageReason);
              double scored = applyHysteresis(preStage, nowSec);
              if (scored > bestEffectiveUtility) {
                bestEffectiveUtility = scored;
                best = preStage;
              }
            }
            continue;
          }

          double marginalValue =
              computeMarginalScoringValue(
                  site,
                  heldPieceId,
                  eff.effectiveBasePoints(),
                  ledger,
                  optimizationObjective,
                  matchTimeRemainingSec);

          if (!Double.isFinite(lanePenalty)) {
            // Scoring lane directly blocked -> propose staging hold pose
            Pose2d hold2d = season.resolveSiteStagingPose(site, ourAlliance);
            double tHold =
                kinematics.estimateTravelTimeSec(egoXy.getDistance(hold2d.getTranslation()));
            double holdUtil = (marginalValue / (tHold + 3.0)) * 0.45;
            Pose3d hold3d =
                new Pose3d(
                    new Translation3d(hold2d.getX(), hold2d.getY(), 0.0),
                    new Rotation3d(0.0, 0.0, hold2d.getRotation().getRadians()));
            TacticalRecommendation cand =
                new TacticalRecommendation(
                    EncounterIntent.HOLD_STAGE,
                    hold3d,
                    site.id(),
                    heldPieceId,
                    holdUtil,
                    tHold,
                    false,
                    null,
                    "Scoring lane to " + site.id() + " blocked; staging at hold pose");
            double scored = applyHysteresis(cand, nowSec);
            if (scored > bestEffectiveUtility) {
              bestEffectiveUtility = scored;
              best = cand;
            }
          } else {
            double cycleTime =
                Math.max(0.25, tDrive + kinematics.nominalScoreCycleSec() + lanePenalty);
            double pps = marginalValue / cycleTime;
            TacticalRecommendation cand =
                new TacticalRecommendation(
                    EncounterIntent.SCORE,
                    scoreWaypointPose,
                    site.id(),
                    heldPieceId,
                    pps,
                    cycleTime,
                    false,
                    null,
                    String.format("Score %s at %s (%.2f eff pts/s)", heldPieceId, site.id(), pps));
            double scored = applyHysteresis(cand, nowSec);
            if (scored > bestEffectiveUtility) {
              bestEffectiveUtility = scored;
              best = cand;
            }
          }
        }
      }
    }

    // 2. Evaluate GATHER for tracked 6-DoF game pieces if robot has remaining capacity
    for (TrackedGamePiece gp : world.gamePieces()) {
      GamePieceSpec spec = season.findGamePiece(gp.pieceId()).orElse(null);
      if (spec == null || !inventory.canHoldAnother(season, spec)) {
        continue;
      }
      Translation2d pieceXy = gp.pose3d().getTranslation().toTranslation2d();
      if (LaneClearanceChecker.isPointRestricted(
          pieceXy,
          season,
          ourAlliance,
          period,
          matchTimeRemainingSec,
          kinematics.bumperRadiusMeters(),
          false)) {
        continue;
      }

      FieldPathfinder.PathResult gatherPath =
          pathfinder.findPath(egoXy, pieceXy, ourAlliance, period, matchTimeRemainingSec);
      double tEgoToPiece = kinematics.estimateTravelTimeSec(gatherPath.totalDistanceMeters());
      if (isLostToArrivalRace(pieceXy, tEgoToPiece, world)) {
        continue;
      }

      // Check 6-DoF tilt tolerance if intake requires orientation alignment
      if (spec.requireOrientationForIntake() && gp.orientationConfidence() > 0.3) {
        double tiltDeg =
            Math.toDegrees(
                Math.hypot(gp.pose3d().getRotation().getX(), gp.pose3d().getRotation().getY()));
        if (tiltDeg > spec.maxIntakeTiltErrorDeg()) {
          continue;
        }
      }

      // Find best downstream scoring value for this piece type
      double bestSiteValue = 0.0;
      double bestPostGatherTime = 2.0;
      for (ScoringSiteSpec site : season.scoringSitesForPiece(spec.id())) {
        if (!site.isUnlimitedCapacity()
            && ledger.getEstimatedSiteCount(site.id()) >= site.siteCapacity()) {
          continue;
        }
        DynamicScoringState.EffectiveSiteScoring eff =
            dyn.evaluateSite(season, site, period, elapsedMatchSec);
        // Even if temporarily inactive, value gathering using base teleop points so robot
        // stockpiles for next shift
        double basePtsForGather = eff.active() ? eff.effectiveBasePoints() : site.pointsFor(period);
        double val =
            computeMarginalScoringValue(
                site,
                spec.id(),
                basePtsForGather,
                ledger,
                optimizationObjective,
                matchTimeRemainingSec);
        Translation2d siteXy =
            season.resolveSiteApproachPose(site, ourAlliance).getTranslation().toTranslation2d();
        double tPost =
            kinematics.estimateTravelTimeSec(pieceXy.getDistance(siteXy))
                + kinematics.nominalScoreCycleSec();
        if (val / Math.max(0.5, tPost) > bestSiteValue / Math.max(0.5, bestPostGatherTime)) {
          bestSiteValue = val;
          bestPostGatherTime = tPost;
        }
      }
      if (bestSiteValue <= 0.0) {
        continue;
      }

      double totalCycle = tEgoToPiece + kinematics.nominalIntakeCycleSec() + bestPostGatherTime;
      double gatherPps = (bestSiteValue / Math.max(0.5, totalCycle)) * gp.existenceConfidence();

      // Align chassis intake yaw to piece 6-DoF Z rotation when required, else face piece from ego
      double intakeYaw =
          spec.requireOrientationForIntake()
              ? gp.pose3d().getRotation().getZ()
              : Math.atan2(pieceXy.getY() - egoXy.getY(), pieceXy.getX() - egoXy.getX());
      Translation3d gatherTargetTranslation =
          gatherPath.detourRequired()
              ? new Translation3d(
                  gatherPath.nextWaypoint().getX(), gatherPath.nextWaypoint().getY(), 0.0)
              : gp.pose3d().getTranslation();
      Pose3d targetPose = new Pose3d(gatherTargetTranslation, new Rotation3d(0.0, 0.0, intakeYaw));

      TacticalRecommendation cand =
          new TacticalRecommendation(
              EncounterIntent.GATHER,
              targetPose,
              "piece:" + gp.trackId(),
              spec.id(),
              gatherPps,
              tEgoToPiece + kinematics.nominalIntakeCycleSec(),
              false,
              null,
              String.format("Gather %s #%d (%.2f eff pts/s)", spec.id(), gp.trackId(), gatherPps));
      double scored = applyHysteresis(cand, nowSec);
      if (scored > bestEffectiveUtility) {
        bestEffectiveUtility = scored;
        best = cand;
      }
    }

    if (best == null) {
      best = TacticalRecommendation.idle(world.egoPose3d(), "No viable scoring or gather targets");
    }
    recordCommitted(best, nowSec);
    return best;
  }

  private double computeMarginalScoringValue(
      ScoringSiteSpec site,
      String pieceId,
      double effectiveBasePoints,
      MatchScoreLedger ledger,
      OptimizationObjective mode,
      double matchTimeRemainingSec) {
    double total = effectiveBasePoints;

    for (ScoringObjectiveSpec obj : season.scoringObjectives()) {
      if (!obj.matches(site, pieceId)) {
        continue;
      }
      int current = ledger.getEstimatedObjectiveProgress(obj.id());
      int remaining = obj.targetCount() - current;
      if (remaining <= 0) {
        continue; // Already completed -> automatic spillover to base points!
      }
      // Reachability gate: do not chase an objective impossible to finish in remaining time
      double minPossibleSec =
          remaining
              * (kinematics.nominalIntakeCycleSec() + kinematics.nominalScoreCycleSec() + 0.8);
      if (matchTimeRemainingSec > 0.0 && minPossibleSec > matchTimeRemainingSec) {
        continue;
      }

      double progressRatio = (double) current / (double) obj.targetCount();
      if (obj.rewardType() == ObjectiveRewardType.BONUS_MATCH_POINTS) {
        total += (obj.rewardValue() / remaining) * (0.6 + 0.6 * progressRatio);
      } else if (obj.rewardType() == ObjectiveRewardType.RANKING_POINT
          && mode == OptimizationObjective.RANKING_POINTS) {
        total +=
            (obj.rewardValue() * RP_PRIORITY_WEIGHT / Math.max(1.0, remaining * 0.5))
                * (1.0 + progressRatio);
      }
    }
    return total;
  }

  private boolean isLostToArrivalRace(Translation2d pieceXy, double tEgoSec, WorldSnapshot world) {
    for (TrackedRobot bot : world.robots()) {
      Translation2d botXy = bot.pose3d().getTranslation().toTranslation2d();
      double botDist = botXy.getDistance(pieceXy);
      if (botDist < 0.25) {
        return true;
      }
      double ux = (pieceXy.getX() - botXy.getX()) / botDist;
      double uy = (pieceXy.getY() - botXy.getY()) / botDist;
      double closingSpeed = bot.vxMps() * ux + bot.vyMps() * uy;

      double effectiveSpeed = closingSpeed > 0.5 ? Math.max(closingSpeed, 2.0) : 2.0;
      double turnaroundPenalty = closingSpeed < -0.5 ? 0.65 : 0.0;
      double tOther = (botDist / effectiveSpeed) + turnaroundPenalty;

      double margin =
          bot.affiliation() == yaece.perception.RobotAffiliation.ALLY
              ? ALLY_DECONFLICT_MARGIN_SEC
              : OPPONENT_RACE_ABORT_MARGIN_SEC;
      if (tOther + margin < tEgoSec) {
        return true;
      }
    }
    return false;
  }

  private double applyHysteresis(TacticalRecommendation cand, double nowSec) {
    boolean sameTarget = cand.intent() == lastIntent && cand.targetId().equals(lastTargetId);
    if (!sameTarget) {
      return cand.utilityScore();
    }
    double bonus = 1.0 + HYSTERESIS_BONUS_RATIO;
    if ((nowSec - lastSwitchTimestampSec) < MIN_COMMIT_DWELL_SEC) {
      bonus += 0.15;
    }
    return cand.utilityScore() * bonus;
  }

  private void recordCommitted(TacticalRecommendation chosen, double nowSec) {
    if (chosen.intent() != lastIntent || !chosen.targetId().equals(lastTargetId)) {
      lastIntent = chosen.intent();
      lastTargetId = chosen.targetId();
      lastSwitchTimestampSec = nowSec;
    }
  }

  public void reset() {
    lastIntent = EncounterIntent.IDLE;
    lastTargetId = "";
    lastSwitchTimestampSec = -10.0;
  }
}
