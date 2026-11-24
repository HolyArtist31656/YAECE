package yaece.tactics;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;
import yaece.season.AllianceSide;
import yaece.season.FieldMirroring;
import yaece.season.MatchPeriod;
import yaece.season.RestrictedZoneSpec;
import yaece.season.SeasonConfig;
import yaece.season.ZoneRuleType;

/**
 * Proactive velocity-dot hazard stow evaluator (inspired by FRC 1678's {@code
 * updateIsNearingTrench} and FRC 2056's polygon zone hood protection).
 *
 * <p>Returns {@code true} whenever the robot is within {@code stowDistanceMeters} of an active
 * {@link ZoneRuleType#SOFT_AVOIDANCE} hazard zone (such as a trench or low bar) AND its velocity
 * vector points toward the hazard ({@code v.dot(robotToHazard) > 0}), or when already inside it.
 */
public final class HazardAutoTuck {
  private HazardAutoTuck() {}

  public static boolean shouldAutoStow(
      Translation2d egoPosition,
      double egoVxMps,
      double egoVyMps,
      SeasonConfig season,
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      double stowDistanceMeters) {
    Objects.requireNonNull(egoPosition, "egoPosition must not be null");
    Objects.requireNonNull(season, "season must not be null");

    for (RestrictedZoneSpec zone : season.restrictedZones()) {
      if (zone.ruleType() != ZoneRuleType.SOFT_AVOIDANCE
          || !zone.isActive(period, matchTimeRemainingSec)) {
        continue;
      }
      List<Translation2d> poly = season.resolvedZonePolygon(zone, alliance);
      if (poly.isEmpty()) {
        continue;
      }
      if (FieldMirroring.polygonContainsPoint(poly, egoPosition)) {
        return true;
      }
      Translation2d centroid = centroidOf(poly);
      double dx = centroid.getX() - egoPosition.getX();
      double dy = centroid.getY() - egoPosition.getY();
      double dist = Math.hypot(dx, dy);
      if (dist <= stowDistanceMeters) {
        double dot = egoVxMps * dx + egoVyMps * dy;
        if (dot > 0.05) {
          return true;
        }
      }
    }
    return false;
  }

  private static Translation2d centroidOf(List<Translation2d> poly) {
    double cx = 0.0;
    double cy = 0.0;
    for (Translation2d p : poly) {
      cx += p.getX();
      cy += p.getY();
    }
    return new Translation2d(cx / poly.size(), cy / poly.size());
  }
}
