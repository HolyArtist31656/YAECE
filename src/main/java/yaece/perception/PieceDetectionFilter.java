package yaece.perception;

import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import yaece.season.FieldMirroring;

/**
 * 4-gate closed-loop vision game-piece target filter (distilled from FRC 1678's {@code
 * DetectionPIDToPoseCommand}):
 *
 * <ol>
 *   <li>Acquisition debounce ({@code 40 ms} default) before accepting a new detection stream,
 *   <li>Optional spatial inclusion polygon filter,
 *   <li>Heading-cone gate ({@code ±60 deg} default) rejecting off-axis / cross-field pieces,
 *   <li>Zero-allocation fixed-window moving average ({@code 15} samples default).
 * </ol>
 */
public final class PieceDetectionFilter {
  private final double debounceSec;
  private final double maxHeadingConeRad;
  private final int windowSize;
  private final double[] xs;
  private final double[] ys;

  private int count = 0;
  private int head = 0;
  private double firstSeenSec = -1.0;

  public PieceDetectionFilter() {
    this(0.04, Math.toRadians(60.0), 15);
  }

  public PieceDetectionFilter(double debounceSec, double maxHeadingConeRad, int windowSize) {
    if (windowSize < 1) {
      throw new IllegalArgumentException("windowSize must be >= 1");
    }
    this.debounceSec = Math.max(0.0, debounceSec);
    this.maxHeadingConeRad = Math.max(0.05, maxHeadingConeRad);
    this.windowSize = windowSize;
    this.xs = new double[windowSize];
    this.ys = new double[windowSize];
  }

  /**
   * Filters a raw field-frame game-piece detection and returns the smoothed target pose once all 4
   * gates pass.
   */
  public Optional<Pose2d> update(
      Translation2d rawPieceFieldPos,
      Pose2d egoPose2d,
      List<Translation2d> optionalInclusionZone,
      double timestampSec) {
    if (rawPieceFieldPos == null || egoPose2d == null) {
      reset();
      return Optional.empty();
    }

    if (optionalInclusionZone != null
        && !optionalInclusionZone.isEmpty()
        && !FieldMirroring.polygonContainsPoint(optionalInclusionZone, rawPieceFieldPos)) {
      return Optional.empty();
    }

    double dx = rawPieceFieldPos.getX() - egoPose2d.getX();
    double dy = rawPieceFieldPos.getY() - egoPose2d.getY();
    double bearingToPiece = Math.atan2(dy, dx);
    double headingErr = Math.abs(wrapRad(bearingToPiece - egoPose2d.getRotation().getRadians()));
    if (headingErr > maxHeadingConeRad) {
      return Optional.empty();
    }

    if (firstSeenSec < 0.0) {
      firstSeenSec = timestampSec;
    }
    xs[head] = rawPieceFieldPos.getX();
    ys[head] = rawPieceFieldPos.getY();
    head = (head + 1) % windowSize;
    if (count < windowSize) {
      count++;
    }

    if ((timestampSec - firstSeenSec) < debounceSec) {
      return Optional.empty();
    }

    double sumX = 0.0;
    double sumY = 0.0;
    for (int i = 0; i < count; i++) {
      sumX += xs[i];
      sumY += ys[i];
    }
    Translation2d avg = new Translation2d(sumX / count, sumY / count);
    double finalBearing = Math.atan2(avg.getY() - egoPose2d.getY(), avg.getX() - egoPose2d.getX());
    return Optional.of(new Pose2d(avg, Rotation2d.fromRadians(finalBearing)));
  }

  public void reset() {
    count = 0;
    head = 0;
    firstSeenSec = -1.0;
  }

  private static double wrapRad(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
