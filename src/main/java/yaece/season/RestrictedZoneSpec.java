package yaece.season;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;

/**
 * Immutable restricted or protected field zone (canonical Blue-origin polygon) with optional match
 * period and endgame time-window activation.
 *
 * @param id unique zone identifier
 * @param ruleType {@link ZoneRuleType#HARD_NO_ENTRY}, {@link ZoneRuleType#NO_OPPONENT_CONTACT}, or
 *     {@link ZoneRuleType#SOFT_AVOIDANCE}
 * @param blueVertices ordered polygon vertices in canonical Blue meters (size &gt;= 3)
 * @param activeInAuto whether this zone rule applies during Auto
 * @param activeInTeleop whether this zone rule applies during Teleop
 * @param activeWhenMatchTimeRemainingSecLessOrEqual if &gt; 0, zone is active only when remaining
 *     match time is &lt;= this threshold (e.g., 30.0 s for Endgame Tower protection); if &lt;= 0,
 *     active for the full enabled period(s)
 * @param safetyBufferMeters extra exterior clearance margin in meters (&gt;= 0)
 */
public record RestrictedZoneSpec(
    String id,
    ZoneRuleType ruleType,
    List<Translation2d> blueVertices,
    boolean activeInAuto,
    boolean activeInTeleop,
    double activeWhenMatchTimeRemainingSecLessOrEqual,
    double safetyBufferMeters) {

  public RestrictedZoneSpec {
    if (id == null || id.isBlank()) {
      throw new IllegalArgumentException("RestrictedZoneSpec.id must be non-blank");
    }
    Objects.requireNonNull(ruleType, "RestrictedZoneSpec.ruleType must not be null");
    Objects.requireNonNull(blueVertices, "RestrictedZoneSpec.blueVertices must not be null");
    blueVertices = List.copyOf(blueVertices);
    if (blueVertices.size() < 3) {
      throw new IllegalArgumentException(
          "RestrictedZoneSpec.blueVertices must contain at least 3 vertices");
    }
    if (!Double.isFinite(activeWhenMatchTimeRemainingSecLessOrEqual)) {
      throw new IllegalArgumentException(
          "RestrictedZoneSpec.activeWhenMatchTimeRemainingSecLessOrEqual must be finite");
    }
    if (!Double.isFinite(safetyBufferMeters) || safetyBufferMeters < 0.0) {
      throw new IllegalArgumentException(
          "RestrictedZoneSpec.safetyBufferMeters must be finite and >= 0");
    }
  }

  /** Returns whether this zone is active for the given period and remaining match time. */
  public boolean isActive(MatchPeriod period, double matchTimeRemainingSec) {
    Objects.requireNonNull(period, "period must not be null");
    boolean periodMatch = (period == MatchPeriod.AUTO) ? activeInAuto : activeInTeleop;
    if (!periodMatch) {
      return false;
    }
    if (activeWhenMatchTimeRemainingSecLessOrEqual > 0.0) {
      return matchTimeRemainingSec <= activeWhenMatchTimeRemainingSecLessOrEqual;
    }
    return true;
  }

  public List<Translation2d> verticesFor(
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    return FieldMirroring.mirrorPolygon(
        blueVertices, alliance, fieldLengthMeters, fieldWidthMeters, mode);
  }

  public boolean containsPoint(
      Translation2d point,
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    return FieldMirroring.polygonContainsPoint(
        verticesFor(alliance, fieldLengthMeters, fieldWidthMeters, mode), point);
  }
}
