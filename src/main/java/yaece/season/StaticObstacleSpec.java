package yaece.season;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;

/** Immutable 2D polygonal field obstacle extruded vertically to {@code heightMeters}. */
public record StaticObstacleSpec(
    String id,
    List<Translation2d> blueVertices,
    double heightMeters,
    boolean blocksVision,
    boolean blocksChassis) {

  public StaticObstacleSpec {
    if (id == null || id.isBlank()) {
      throw new IllegalArgumentException("StaticObstacleSpec.id must be non-blank");
    }
    Objects.requireNonNull(blueVertices, "StaticObstacleSpec.blueVertices must not be null");
    blueVertices = List.copyOf(blueVertices);
    if (blueVertices.size() < 3) {
      throw new IllegalArgumentException(
          "StaticObstacleSpec.blueVertices must contain at least 3 vertices");
    }
    if (!Double.isFinite(heightMeters) || heightMeters <= 0.0) {
      throw new IllegalArgumentException("StaticObstacleSpec.heightMeters must be finite and > 0");
    }
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
