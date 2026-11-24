package yaece.tactics;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.ledger.DefenseModeResolver;
import yaece.ledger.ScoreProjection;
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

  private final SeasonConfig season;
  private final RobotKinematicsSpec kinematics;

  public DefenseInterceptor(SeasonConfig season, RobotKinematicsSpec kinematics) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.kinematics = Objects.requireNonNull(kinematics, "kinematics must not be null");
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

      Translation2d oppXy = opp.pose3d().getTranslation().toTranslation2d();
      ScoringSiteSpec inferredSite = inferOpponentTargetSite(opp, oppAlliance, period);
      Translation2d goalXy =
          inferredSite != null
              ? season
                  .resolveSiteApproachPose(inferredSite, oppAlliance)
                  .getTranslation()
                  .toTranslation2d()
              : opp.predictPositionAt(1.2);
      double sitePts = inferredSite != null ? inferredSite.pointsFor(period) : 2.0;

      double oppDistToGoal = Math.max(0.5, oppXy.getDistance(goalXy));
      double oppSpeed = Math.max(1.5, opp.speedMps());
      double threat = sitePts / (oppDistToGoal / oppSpeed + 0.5);

      if (threat > bestThreat) {
        Translation2d rawIntercept = solveKinematicIntercept(egoXy, oppXy, goalXy, oppSpeed);
        Translation2d clamped =
            LaneClearanceChecker.clampOutsideRestrictedZones(
                rawIntercept,
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
        "Blocking opponent #" + bestOpp.trackId() + " heading toward " + bestSiteId);
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
      List<Translation2d> poly =
          FieldMirroring.mirrorPolygon(
              zone.blueVertices(),
              ourAlliance,
              season.fieldLengthMeters(),
              season.fieldWidthMeters(),
              season.mirrorMode());
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
      TrackedRobot opp, AllianceSide oppAlliance, MatchPeriod period) {
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
      double score = (site.pointsFor(period) * alignment) / (dist + 1.0);
      if (score > bestScore) {
        bestScore = score;
        best = site;
      }
    }
    return best;
  }

  private Translation2d solveKinematicIntercept(
      Translation2d egoXy, Translation2d oppXy, Translation2d goalXy, double oppSpeedMps) {
    double totalOppDist = oppXy.getDistance(goalXy);
    for (int i = 2; i <= 10; i++) {
      double s = i / 10.0;
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
    // Fallback: guard 1.0 m in front of the goal along the opponent's approach line
    return new Translation2d(
        oppXy.getX() + 0.85 * (goalXy.getX() - oppXy.getX()),
        oppXy.getY() + 0.85 * (goalXy.getY() - oppXy.getY()));
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
}
