package yaece.perception;

import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;
import yaece.season.SeasonConfig;
import yaece.world.EgoPoseBuffer;

/**
 * Sensor-agnostic vision localization rejection gate and dynamic standard-deviation scaler
 * combining verified FRC 254, 6328 (2026), and 1678 (2025–2026) practices.
 */
public final class VisionPoseGate {
  /** Operating mode controlling linear XY standard deviation scaling (FRC 1678 2026 pattern). */
  public enum FusionMode {
    DEFAULT(0.30),
    ALIGN(0.10),
    POST_BUMP_SNAP(0.02);

    private final double baseXyStdDev;

    FusionMode(double baseXyStdDev) {
      this.baseXyStdDev = baseXyStdDev;
    }

    public double baseXyStdDev() {
      return baseXyStdDev;
    }
  }

  public record GateResult(
      boolean accepted, double xyStdDevMeters, double thetaStdDevRad, String reason) {}

  private final SeasonConfig season;
  private final double maxYawRateRadPerSec;
  private final double rateWindowHalfSec;
  private final double maxSingleTagAmbiguity;
  private final double minSingleTagArea;
  private final double minZMeters;
  private final double maxZMeters;
  private final double fieldBorderMarginMeters;
  private final double maxUnverifiedJumpMeters;

  public VisionPoseGate(SeasonConfig season) {
    this(season, 5.0, 0.30, 0.19, 0.10, -0.50, 1.00, 0.50, 1.00);
  }

  public VisionPoseGate(
      SeasonConfig season,
      double maxYawRateRadPerSec,
      double rateWindowHalfSec,
      double maxSingleTagAmbiguity,
      double minSingleTagArea,
      double minZMeters,
      double maxZMeters,
      double fieldBorderMarginMeters,
      double maxUnverifiedJumpMeters) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.maxYawRateRadPerSec = maxYawRateRadPerSec;
    this.rateWindowHalfSec = rateWindowHalfSec;
    this.maxSingleTagAmbiguity = maxSingleTagAmbiguity;
    this.minSingleTagArea = minSingleTagArea;
    this.minZMeters = minZMeters;
    this.maxZMeters = maxZMeters;
    this.fieldBorderMarginMeters = fieldBorderMarginMeters;
    this.maxUnverifiedJumpMeters = maxUnverifiedJumpMeters;
  }

  public GateResult evaluate(
      Pose3d visionPose,
      int tagCount,
      double avgTagDistanceMeters,
      double avgTagArea,
      double singleTagAmbiguity,
      double captureTimestampSec,
      EgoPoseBuffer egoBuffer,
      FusionMode mode,
      boolean trustLargeJump) {
    if (visionPose == null || tagCount <= 0 || captureTimestampSec <= 0.0) {
      return new GateResult(
          false,
          Double.POSITIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          "No tags or invalid timestamp");
    }

    double x = visionPose.getX();
    double y = visionPose.getY();
    double z = visionPose.getZ();
    if (x < -fieldBorderMarginMeters
        || x > season.fieldLengthMeters() + fieldBorderMarginMeters
        || y < -fieldBorderMarginMeters
        || y > season.fieldWidthMeters() + fieldBorderMarginMeters) {
      return new GateResult(
          false, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, "Pose outside field bounds");
    }

    if (z < minZMeters || z > maxZMeters) {
      return new GateResult(
          false,
          Double.POSITIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          String.format("Z=%.2fm out of bounds", z));
    }

    if (tagCount == 1) {
      if (singleTagAmbiguity > maxSingleTagAmbiguity) {
        return new GateResult(
            false,
            Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            "Single-tag ambiguity too high");
      }
      if (avgTagArea < minSingleTagArea) {
        return new GateResult(
            false, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, "Single-tag area too small");
      }
    }

    if (egoBuffer != null) {
      double peakYawRate =
          egoBuffer.maxAbsYawRateInWindow(
              captureTimestampSec - rateWindowHalfSec, captureTimestampSec + rateWindowHalfSec);
      if (peakYawRate > maxYawRateRadPerSec) {
        return new GateResult(
            false,
            Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            String.format(
                "Windowed yaw rate %.2f rad/s > %.2f rad/s", peakYawRate, maxYawRateRadPerSec));
      }

      if (!trustLargeJump) {
        var hist = egoBuffer.sampleAt(captureTimestampSec);
        if (hist.isPresent()) {
          double jump =
              hist.get()
                  .getTranslation()
                  .toTranslation2d()
                  .getDistance(visionPose.getTranslation().toTranslation2d());
          if (jump > maxUnverifiedJumpMeters && tagCount < 2) {
            return new GateResult(
                false,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                String.format("Unverified single-tag jump %.2fm", jump));
          }
        }
      }
    }

    double distFactor = Math.max(0.5, avgTagDistanceMeters);
    double tagDiv = tagCount >= 3 ? 2.5 : (tagCount == 2 ? 1.5 : 1.0);
    double xyStd = mode.baseXyStdDev() * distFactor / tagDiv;
    // Gyro remains authoritative for heading (thetaStdDev = +Inf)
    return new GateResult(true, xyStd, Double.POSITIVE_INFINITY, "Accepted (" + mode + ")");
  }
}
