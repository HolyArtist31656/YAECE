// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.ledger.DefenseModeResolver;
import yaece.ledger.ScoreProjection;
import yaece.scouting.OpponentProfile;
import yaece.scouting.ScoutingPriorRegistry;
import yaece.season.AllianceSide;
import yaece.season.DefenseConfigSpec;
import yaece.season.DefenseStrategyMode;
import yaece.season.DefenseZoneSpec;
import yaece.season.FieldMirroring;
import yaece.season.MatchPeriod;
import yaece.season.ScoringSiteSpec;
import yaece.season.SeasonConfig;
import yaece.world.OpponentPinState;
import yaece.world.PinStatus;
import yaece.world.TrackedRobot;
import yaece.world.WorldSnapshot;

/**
 * Computes optimal legal defensive blocking and pin-retreat poses from opponent velocities,
 * mirrored opponent scoring sites, restricted zones, and live score margins.
 */
public final class DefenseInterceptor {
  private static final double INTERCEPT_LEAD_MARGIN_SEC = 0.25;
  private static final double SCOUTED_CRUISE_SPEED_RATIO = 0.75;
  private static final double MIN_INTERCEPT_SPEED_MPS = 1.5;
  private static final double INTERCEPT_STANDOFF_MARGIN_METERS = 0.15;
  private static final double ALLY_CORRIDOR_LOOKAHEAD_SEC = 1.0;
  private static final double ALLY_KEEP_OUT_MARGIN_METERS = 0.20;
  private static final double MIN_DIRECTION_NORM = 1e-4;

  private final SeasonConfig season;
  private final RobotKinematicsSpec kinematics;
  private final ScoutingPriorRegistry scoutingPriors;

  public DefenseInterceptor(SeasonConfig season, RobotKinematicsSpec kinematics) {
    this(season, kinematics, new ScoutingPriorRegistry());
  }

  public DefenseInterceptor(
      SeasonConfig season, RobotKinematicsSpec kinematics, ScoutingPriorRegistry scoutingPriors) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.kinematics = Objects.requireNonNull(kinematics, "kinematics must not be null");
    this.scoutingPriors = Objects.requireNonNull(scoutingPriors, "scoutingPriors must not be null");
  }

  /** Evaluates the best legal defensive action for the current tick. */
  public TacticalRecommendation evaluate(
      WorldSnapshot world,
      AllianceSide ourAlliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      DefenseStrategyMode requestedMode,
      ScoreProjection scoreProjection) {
    DefenseConfigSpec defCfg = season.defenseDefaults();
    DefenseStrategyMode resolved =
        DefenseModeResolver.resolve(requestedMode, defCfg, scoreProjection);

    Translation2d egoXy = world.egoPose3d().getTranslation().toTranslation2d();
    List<TrackedRobot> opponents = world.opponents();

    // 1. Hard FRC Pin-Guard priority: if any opponent requires backoff, retreat >= separation +
    // 0.25m
    for (Map.Entry<Integer, OpponentPinState> entry : world.pinStates().entrySet()) {
      OpponentPinState pin = entry.getValue();
      if (pin.backoffRequired()) {
        TrackedRobot pinnedBot = findOpponentById(opponents, pin.trackId());
        if (pinnedBot != null) {
          Translation2d oppXy = pinnedBot.pose3d().getTranslation().toTranslation2d();
          Translation2d retreatXy =
              computePinRetreatPoint(egoXy, oppXy, ourAlliance, period, matchTimeRemainingSec);
          double yaw = Math.atan2(oppXy.getY() - retreatXy.getY(), oppXy.getX() - retreatXy.getX());
          Pose3d target =
              new Pose3d(
                  new Translation3d(retreatXy.getX(), retreatXy.getY(), 0.0),
                  new Rotation3d(0, 0, yaw));
          return new TacticalRecommendation(
              EncounterIntent.DEFEND_PIN_RETREAT,
              target,
              "opponent:" + pin.trackId(),
              "",
              100.0,
              kinematics.estimateTravelTimeSec(egoXy.getDistance(retreatXy)),
              kinematics.stowMechanismsInDefense(),
              resolved,
              String.format(
                  "Pin guard backoff (%.2fs active, %.2fm gap)",
                  pin.activePinSeconds(), pin.separationMeters()));
        }
      }
    }

    // 2. Zone Defense mode (if requested/resolved and zones exist)
    if (resolved == DefenseStrategyMode.ZONE_DEFENSE && !defCfg.defenseZones().isEmpty()) {
      return evaluateZoneDefense(
          world, egoXy, opponents, ourAlliance, period, matchTimeRemainingSec, defCfg);
    }

    // 3. Highest-Threat Interception mode
    return evaluateHighestThreat(
        world, egoXy, opponents, ourAlliance, period, matchTimeRemainingSec, resolved);
  }

  private TacticalRecommendation evaluateHighestThreat(
      WorldSnapshot world,
      Translation2d egoXy,
      List<TrackedRobot> opponents,
      AllianceSide ourAlliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      DefenseStrategyMode resolved) {
    AllianceSide oppAlliance = ourAlliance.opposite();
    TrackedRobot bestOpp = null;
    Translation2d bestIntercept = null;
    double bestThreat = -1.0;
    String bestSiteId = "";
    double bestYaw = 0.0;

    for (TrackedRobot opp : opponents) {
      OpponentPinState pin = world.pinStates().get(opp.trackId());
      if (pin != null && pin.status() == PinStatus.COOLDOWN_PAUSED) {
        continue; // Skip opponent while cooling down from a prior pin
      }

      OpponentProfile profile = scoutingPriors.resolveOrDefaultForTrack(opp.trackId());
      Translation2d oppXy = opp.pose3d().getTranslation().toTranslation2d();
      ScoringSiteSpec inferredSite = inferOpponentTargetSite(opp, profile, oppAlliance, period);
      Translation2d goalXy =
          inferredSite != null
              ? season
                  .resolveSiteApproachPose(inferredSite, oppAlliance)
                  .getTranslation()
                  .toTranslation2d()
              : opp.predictPositionAt(1.2);
      double sitePts = inferredSite != null ? inferredSite.pointsFor(period) : 2.0;

      double oppDistToGoal = Math.max(0.5, oppXy.getDistance(goalXy));
      double scoutedSpeedFloor =
          Math.max(MIN_INTERCEPT_SPEED_MPS, SCOUTED_CRUISE_SPEED_RATIO * profile.maxSpeedMps());
      double oppSpeed =
          Math.max(scoutedSpeedFloor, Math.min(opp.speedMps(), profile.maxSpeedMps()));
      String siteId = inferredSite != null ? inferredSite.id() : "";
      double scoutingWeight =
          scoutingPriors.threatMultiplierFor(opp.trackId(), period, matchTimeRemainingSec, siteId);
      double threat = (sitePts / (oppDistToGoal / oppSpeed + 0.5)) * scoutingWeight;

      if (threat > bestThreat) {
        double standoffMeters =
            kinematics.bumperRadiusMeters()
                + profile.bumperRadiusMeters()
                + INTERCEPT_STANDOFF_MARGIN_METERS;
        Translation2d rawIntercept =
            solveKinematicIntercept(egoXy, oppXy, goalXy, oppSpeed, standoffMeters);
        Translation2d allySafe =
            applyAllyCorridorKeepOut(
                rawIntercept, world.allies(), egoXy, ourAlliance, period, matchTimeRemainingSec);
        Translation2d clamped =
            LaneClearanceChecker.clampOutsideRestrictedZones(
                allySafe,
                egoXy,
                season,
                ourAlliance,
                period,
                matchTimeRemainingSec,
                kinematics.bumperRadiusMeters());
        // Orient broadside to opponent approach vector
        double approachAngle =
            Math.atan2(goalXy.getY() - oppXy.getY(), goalXy.getX() - oppXy.getX());
        bestThreat = threat;
        bestOpp = opp;
        bestIntercept = clamped;
        bestSiteId = inferredSite != null ? inferredSite.id() : "velocity_ray";
        bestYaw = approachAngle + (Math.PI / 2.0);
      }
    }

    if (bestOpp == null || bestIntercept == null) {
      return TacticalRecommendation.idle(world.egoPose3d(), "No eligible opponents to defend");
    }

    Pose3d target =
        new Pose3d(
            new Translation3d(bestIntercept.getX(), bestIntercept.getY(), 0.0),
            new Rotation3d(0.0, 0.0, bestYaw));
    return new TacticalRecommendation(
        EncounterIntent.DEFEND_INTERCEPT,
        target,
        "opponent:" + bestOpp.trackId(),
        "",
        bestThreat,
        kinematics.estimateTravelTimeSec(egoXy.getDistance(bestIntercept)),
        kinematics.stowMechanismsInDefense(),
        resolved,
        formatInterceptReason(bestOpp.trackId(), bestSiteId));
  }

  private TacticalRecommendation evaluateZoneDefense(
      WorldSnapshot world,
      Translation2d egoXy,
      List<TrackedRobot> opponents,
      AllianceSide ourAlliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      DefenseConfigSpec defCfg) {
    DefenseZoneSpec bestZone = defCfg.defenseZones().get(0);
    double bestScore = -1.0;
    Translation2d interceptPt = null;

    for (DefenseZoneSpec zone : defCfg.defenseZones()) {
      List<Translation2d> poly = season.resolvedDefenseZonePolygon(zone, ourAlliance);
      Pose2d anchor =
          zone.anchorPoseFor(
              ourAlliance,
              season.fieldLengthMeters(),
              season.fieldWidthMeters(),
              season.mirrorMode());

      double zoneScore = zone.priorityWeight();
      Translation2d candidatePt = anchor.getTranslation();
      for (TrackedRobot opp : opponents) {
        Translation2d oppPred = opp.predictPositionAt(0.75);
        if (FieldMirroring.polygonContainsPoint(poly, oppPred)) {
          zoneScore += 5.0 * zone.priorityWeight();
          candidatePt = oppPred;
        }
      }
      if (zoneScore > bestScore) {
        bestScore = zoneScore;
        bestZone = zone;
        interceptPt = candidatePt;
      }
    }

    Translation2d clamped =
        LaneClearanceChecker.clampOutsideRestrictedZones(
            interceptPt,
            egoXy,
            season,
            ourAlliance,
            period,
            matchTimeRemainingSec,
            kinematics.bumperRadiusMeters());
    Pose2d anchor =
        bestZone.anchorPoseFor(
            ourAlliance,
            season.fieldLengthMeters(),
            season.fieldWidthMeters(),
            season.mirrorMode());
    Pose3d target =
        new Pose3d(
            new Translation3d(clamped.getX(), clamped.getY(), 0.0),
            new Rotation3d(0.0, 0.0, anchor.getRotation().getRadians()));

    return new TacticalRecommendation(
        EncounterIntent.DEFEND_INTERCEPT,
        target,
        "zone:" + bestZone.id(),
        "",
        bestScore,
        kinematics.estimateTravelTimeSec(egoXy.getDistance(clamped)),
        kinematics.stowMechanismsInDefense(),
        DefenseStrategyMode.ZONE_DEFENSE,
        "Zone defense holding corridor " + bestZone.id());
  }

  private ScoringSiteSpec inferOpponentTargetSite(
      TrackedRobot opp, OpponentProfile profile, AllianceSide oppAlliance, MatchPeriod period) {
    Translation2d oppXy = opp.pose3d().getTranslation().toTranslation2d();
    double speed = opp.speedMps();
    ScoringSiteSpec best = null;
    double bestScore = -Double.MAX_VALUE;

    for (ScoringSiteSpec site : season.scoringSites()) {
      Translation2d siteXy =
          season.resolveSiteApproachPose(site, oppAlliance).getTranslation().toTranslation2d();
      double dx = siteXy.getX() - oppXy.getX();
      double dy = siteXy.getY() - oppXy.getY();
      double dist = Math.max(0.25, Math.hypot(dx, dy));

      double alignment = 0.5;
      if (speed > 0.25) {
        double cosTheta = (opp.vxMps() * dx + opp.vyMps() * dy) / (speed * dist);
        alignment = Math.max(0.05, 0.5 * (cosTheta + 1.0));
      }
      double sitePref = profile.sitePreferenceMultiplier(site.id());
      double score = (site.pointsFor(period) * alignment * sitePref) / (dist + 1.0);
      if (score > bestScore) {
        bestScore = score;
        best = site;
      }
    }
    return best;
  }

  private Translation2d solveKinematicIntercept(
      Translation2d egoXy,
      Translation2d oppXy,
      Translation2d goalXy,
      double oppSpeedMps,
      double standoffMeters) {
    double totalOppDist = oppXy.getDistance(goalXy);
    double minS =
        totalOppDist > MIN_DIRECTION_NORM ? Math.min(0.85, standoffMeters / totalOppDist) : 0.2;
    for (int i = 2; i <= 10; i++) {
      double s = Math.max(minS, i / 10.0);
      Translation2d pt =
          new Translation2d(
              oppXy.getX() + s * (goalXy.getX() - oppXy.getX()),
              oppXy.getY() + s * (goalXy.getY() - oppXy.getY()));
      double tEgo = kinematics.estimateTravelTimeSec(egoXy.getDistance(pt));
      double tOpp = (s * totalOppDist) / Math.max(0.5, oppSpeedMps);
      if (tEgo <= tOpp - INTERCEPT_LEAD_MARGIN_SEC) {
        return pt;
      }
    }
    return new Translation2d(
        oppXy.getX() + 0.85 * (goalXy.getX() - oppXy.getX()),
        oppXy.getY() + 0.85 * (goalXy.getY() - oppXy.getY()));
  }

  private Translation2d applyAllyCorridorKeepOut(
      Translation2d candidate,
      List<TrackedRobot> allies,
      Translation2d egoXy,
      AllianceSide ourAlliance,
      MatchPeriod period,
      double matchTimeRemainingSec) {
    if (allies.isEmpty()) {
      return candidate;
    }
    Translation2d adjusted = candidate;
    for (TrackedRobot ally : allies) {
      OpponentProfile allyProfile = scoutingPriors.resolveOrDefaultForTrack(ally.trackId());
      double clearance =
          kinematics.bumperRadiusMeters()
              + allyProfile.bumperRadiusMeters()
              + ALLY_KEEP_OUT_MARGIN_METERS;
      Translation2d allyStart = ally.pose3d().getTranslation().toTranslation2d();
      Translation2d allyEnd = ally.predictPositionAt(ALLY_CORRIDOR_LOOKAHEAD_SEC);
      Translation2d closest = closestPointOnSegment(adjusted, allyStart, allyEnd);
      double dist = adjusted.getDistance(closest);
      if (dist >= clearance) {
        continue;
      }
      double vx = allyEnd.getX() - allyStart.getX();
      double vy = allyEnd.getY() - allyStart.getY();
      double vLen = Math.hypot(vx, vy);
      double nx;
      double ny;
      if (vLen > MIN_DIRECTION_NORM) {
        // Lateral normal perpendicular to the ally's travel corridor
        nx = -vy / vLen;
        ny = vx / vLen;
      } else {
        double dx = adjusted.getX() - allyStart.getX();
        double dy = adjusted.getY() - allyStart.getY();
        double dLen = Math.hypot(dx, dy);
        if (dLen > MIN_DIRECTION_NORM) {
          nx = dx / dLen;
          ny = dy / dLen;
        } else {
          nx = 0.0;
          ny = 1.0;
        }
      }
      double push = (clearance - dist) + 0.05;
      Translation2d optionA =
          new Translation2d(
              clampFieldX(adjusted.getX() + nx * push), clampFieldY(adjusted.getY() + ny * push));
      Translation2d optionB =
          new Translation2d(
              clampFieldX(adjusted.getX() - nx * push), clampFieldY(adjusted.getY() - ny * push));
      Translation2d clampedA =
          LaneClearanceChecker.clampOutsideRestrictedZones(
              optionA,
              egoXy,
              season,
              ourAlliance,
              period,
              matchTimeRemainingSec,
              kinematics.bumperRadiusMeters());
      Translation2d clampedB =
          LaneClearanceChecker.clampOutsideRestrictedZones(
              optionB,
              egoXy,
              season,
              ourAlliance,
              period,
              matchTimeRemainingSec,
              kinematics.bumperRadiusMeters());
      double distA = clampedA.getDistance(closest);
      double distB = clampedB.getDistance(closest);
      adjusted = distA >= distB ? clampedA : clampedB;
    }
    return adjusted;
  }

  private static Translation2d closestPointOnSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    double abx = b.getX() - a.getX();
    double aby = b.getY() - a.getY();
    double abLenSq = abx * abx + aby * aby;
    if (abLenSq <= 1e-9) {
      return a;
    }
    double t = ((p.getX() - a.getX()) * abx + (p.getY() - a.getY()) * aby) / abLenSq;
    t = Math.max(0.0, Math.min(1.0, t));
    return new Translation2d(a.getX() + t * abx, a.getY() + t * aby);
  }

  private double clampFieldX(double x) {
    double margin = kinematics.bumperRadiusMeters();
    return Math.max(margin, Math.min(season.fieldLengthMeters() - margin, x));
  }

  private double clampFieldY(double y) {
    double margin = kinematics.bumperRadiusMeters();
    return Math.max(margin, Math.min(season.fieldWidthMeters() - margin, y));
  }

  private Translation2d computePinRetreatPoint(
      Translation2d egoXy,
      Translation2d oppXy,
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec) {
    double dx = egoXy.getX() - oppXy.getX();
    double dy = egoXy.getY() - oppXy.getY();
    double len = Math.hypot(dx, dy);
    if (len < 1e-4) {
      dx = -1.0;
      dy = 0.0;
      len = 1.0;
    }
    double targetCenterDist =
        season.pinRules().separationDistanceMeters() + 2.0 * kinematics.bumperRadiusMeters() + 0.25;
    Translation2d raw =
        new Translation2d(
            oppXy.getX() + (dx / len) * targetCenterDist,
            oppXy.getY() + (dy / len) * targetCenterDist);
    return LaneClearanceChecker.clampOutsideRestrictedZones(
        raw,
        egoXy,
        season,
        alliance,
        period,
        matchTimeRemainingSec,
        kinematics.bumperRadiusMeters());
  }

  private static TrackedRobot findOpponentById(List<TrackedRobot> opponents, int trackId) {
    for (TrackedRobot bot : opponents) {
      if (bot.trackId() == trackId) {
        return bot;
      }
    }
    return null;
  }

  private String formatInterceptReason(int trackId, String siteId) {
    Optional<OpponentProfile> profile = scoutingPriors.resolveForTrack(trackId);
    if (profile.isPresent() && profile.get().teamNumber() > 0) {
      OpponentProfile p = profile.get();
      return "Blocking opponent #"
          + trackId
          + " (Team "
          + p.teamNumber()
          + ", "
          + p.rolePrior()
          + ") heading toward "
          + siteId;
    }
    return "Blocking opponent #" + trackId + " heading toward " + siteId;
  }
}
