package yaece.tactics;

import java.util.List;
import org.wpilib.math.geometry.Translation2d;
import yaece.perception.RobotAffiliation;
import yaece.season.AllianceSide;
import yaece.season.FieldMirroring;
import yaece.season.MatchPeriod;
import yaece.season.RestrictedZoneSpec;
import yaece.season.SeasonConfig;
import yaece.season.StaticObstacleSpec;
import yaece.season.ZoneRuleType;
import yaece.world.TrackedRobot;
import yaece.world.WorldSnapshot;

/**
 * Geometric clearance, restricted-zone clamping, and dynamic lane-congestion checker for both
 * offensive scoring lanes and defensive blocking poses.
 */
public final class LaneClearanceChecker {
  private LaneClearanceChecker() {}

  /**
   * Returns true if {@code point} violates any active {@link ZoneRuleType#HARD_NO_ENTRY} zone or
   * chassis-blocking {@link StaticObstacleSpec}, or if {@code checkNoContact} is true, falls inside
   * an active {@link ZoneRuleType#NO_OPPONENT_CONTACT} zone (including its safety buffer).
   */
  public static boolean isPointRestricted(
      Translation2d point,
      SeasonConfig season,
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      double egoBumperRadiusMeters,
      boolean checkNoContact) {
    for (StaticObstacleSpec obs : season.staticObstacles()) {
      if (!obs.blocksChassis()) {
        continue;
      }
      List<Translation2d> poly = season.resolvedObstaclePolygon(obs, alliance);
      if (FieldMirroring.polygonContainsPoint(poly, point)
          || distanceToPolygonEdges(point, poly) < egoBumperRadiusMeters) {
        return true;
      }
    }

    for (RestrictedZoneSpec zone : season.restrictedZones()) {
      if (!zone.isActive(period, matchTimeRemainingSec)) {
        continue;
      }
      if (zone.ruleType() == ZoneRuleType.SOFT_AVOIDANCE) {
        continue;
      }
      if (zone.ruleType() == ZoneRuleType.NO_OPPONENT_CONTACT && !checkNoContact) {
        continue;
      }
      List<Translation2d> poly = season.resolvedZonePolygon(zone, alliance);
      double pad = egoBumperRadiusMeters + zone.safetyBufferMeters();
      if (FieldMirroring.polygonContainsPoint(poly, point)
          || distanceToPolygonEdges(point, poly) < pad) {
        return true;
      }
    }
    return false;
  }

  /**
   * Clamps a desired defensive blocking point cleanly outside any active {@code HARD_NO_ENTRY} or
   * {@code NO_OPPONENT_CONTACT} zone by stepping along the vector toward {@code fallbackOutside}.
   */
  public static Translation2d clampOutsideRestrictedZones(
      Translation2d desired,
      Translation2d fallbackOutside,
      SeasonConfig season,
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      double egoBumperRadiusMeters) {
    if (!isPointRestricted(
        desired, season, alliance, period, matchTimeRemainingSec, egoBumperRadiusMeters, true)) {
      return clampToField(desired, season, egoBumperRadiusMeters);
    }

    // Binary/step search along segment from desired -> fallbackOutside to stay right on the legal
    // edge
    for (int step = 1; step <= 20; step++) {
      double alpha = step / 20.0;
      Translation2d candidate =
          new Translation2d(
              desired.getX() + alpha * (fallbackOutside.getX() - desired.getX()),
              desired.getY() + alpha * (fallbackOutside.getY() - desired.getY()));
      if (!isPointRestricted(
          candidate,
          season,
          alliance,
          period,
          matchTimeRemainingSec,
          egoBumperRadiusMeters,
          true)) {
        return clampToField(candidate, season, egoBumperRadiusMeters);
      }
    }
    return clampToField(fallbackOutside, season, egoBumperRadiusMeters);
  }

  /**
   * Evaluates congestion/blocking along the scoring lane from {@code start} to {@code goal} at
   * lookahead {@code arrivalTimeSec}. Returns {@link Double#POSITIVE_INFINITY} if the goal pose is
   * directly occupied by another robot, or a non-negative time penalty in seconds otherwise.
   */
  public static double evaluateLaneCongestionPenaltySec(
      Translation2d start,
      Translation2d goal,
      double arrivalTimeSec,
      WorldSnapshot world,
      double egoBumperRadiusMeters) {
    double penaltySec = 0.0;
    double lookahead = Math.min(1.5, Math.max(0.1, arrivalTimeSec));

    for (TrackedRobot bot : world.robots()) {
      Translation2d predPos = bot.predictPositionAt(lookahead);
      double clearRadius =
          egoBumperRadiusMeters
              + bot.estimatedRadiusMeters()
              + Math.min(0.6, bot.predictUncertaintyRadiusAt(lookahead));

      // Goal directly occupied?
      double distToGoal = predPos.getDistance(goal);
      if (distToGoal < clearRadius) {
        return Double.POSITIVE_INFINITY;
      }

      // Robot crossing our direct path?
      double segDist = distancePointToSegment(predPos, start, goal);
      if (segDist < clearRadius) {
        double weight = bot.affiliation() == RobotAffiliation.OPPONENT ? 1.4 : 0.8;
        penaltySec += weight * (1.0 - segDist / clearRadius) * 1.5;
      }
    }
    return penaltySec;
  }

  private static Translation2d clampToField(Translation2d p, SeasonConfig season, double margin) {
    double x = Math.max(margin, Math.min(season.fieldLengthMeters() - margin, p.getX()));
    double y = Math.max(margin, Math.min(season.fieldWidthMeters() - margin, p.getY()));
    return new Translation2d(x, y);
  }

  private static double distanceToPolygonEdges(Translation2d p, List<Translation2d> poly) {
    double best = Double.POSITIVE_INFINITY;
    int n = poly.size();
    for (int i = 0; i < n; i++) {
      double d = distancePointToSegment(p, poly.get(i), poly.get((i + 1) % n));
      if (d < best) {
        best = d;
      }
    }
    return best;
  }

  private static double distancePointToSegment(Translation2d p, Translation2d a, Translation2d b) {
    double dx = b.getX() - a.getX();
    double dy = b.getY() - a.getY();
    double lenSq = dx * dx + dy * dy;
    if (lenSq < 1e-9) {
      return p.getDistance(a);
    }
    double t = ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / lenSq;
    t = Math.max(0.0, Math.min(1.0, t));
    double projX = a.getX() + t * dx;
    double projY = a.getY() + t * dy;
    return Math.hypot(p.getX() - projX, p.getY() - projY);
  }
}
