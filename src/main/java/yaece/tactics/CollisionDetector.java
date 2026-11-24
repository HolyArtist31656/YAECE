package yaece.tactics;

/**
 * Speed-ratio midfield collision and trajectory-stall detector (inspired by FRC 1678's {@code
 * cmdWithCollisonAvoidance} and FRC 1690's collision recovery).
 *
 * <p>Flags a collision when {@code actualSpeed / expectedSpeed <= ratioThreshold} (default {@code
 * 0.45}) continuously for {@code debounceSec} (default {@code 0.15 s}) while farther than {@code
 * minGoalDistanceMeters} (default {@code 1.0 m}) from the goal.
 */
public final class CollisionDetector {
  private final double speedRatioThreshold;
  private final double debounceSec;
  private final double minGoalDistanceMeters;
  private final double minExpectedSpeedMps;

  private double stalledDurationSec = 0.0;
  private double lastTimestampSec = -1.0;

  public CollisionDetector() {
    this(0.45, 0.15, 1.0, 0.75);
  }

  public CollisionDetector(
      double speedRatioThreshold,
      double debounceSec,
      double minGoalDistanceMeters,
      double minExpectedSpeedMps) {
    this.speedRatioThreshold = speedRatioThreshold;
    this.debounceSec = debounceSec;
    this.minGoalDistanceMeters = minGoalDistanceMeters;
    this.minExpectedSpeedMps = minExpectedSpeedMps;
  }

  /**
   * Updates the detector for the current tick and returns {@code true} if collision recovery should
   * trigger.
   */
  public boolean update(
      double timestampSec,
      double actualSpeedMps,
      double expectedSpeedMps,
      double distanceToGoalMeters) {
    double dt =
        lastTimestampSec < 0.0
            ? 0.02
            : Math.max(0.0, Math.min(0.25, timestampSec - lastTimestampSec));
    lastTimestampSec = timestampSec;

    if (distanceToGoalMeters <= minGoalDistanceMeters || expectedSpeedMps < minExpectedSpeedMps) {
      stalledDurationSec = 0.0;
      return false;
    }

    double ratio = actualSpeedMps / expectedSpeedMps;
    if (ratio <= speedRatioThreshold) {
      stalledDurationSec += dt;
    } else {
      stalledDurationSec = 0.0;
    }
    return stalledDurationSec >= debounceSec;
  }

  public void reset() {
    stalledDurationSec = 0.0;
    lastTimestampSec = -1.0;
  }
}
