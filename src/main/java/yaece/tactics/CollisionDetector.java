// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

/**
 * Autonomous trajectory-stall and midfield collision-recovery detector (inspired by FRC 1678's
 * {@code cmdWithCollisonAvoidance} and FRC 1690's collision recovery).
 *
 * <p>This class monitors tactical path progress ({@code actualSpeed / expectedSpeed}) to trigger
 * autonomous re-planning or recovery. Hardware-level IMU jerk and impact monitoring belongs in YAMS
 * diagnostics rather than YAECE.
 *
 * <p>Flags a trajectory stall when {@code actualSpeed / expectedSpeed <= ratioThreshold} (default
 * {@code 0.45}) continuously for {@code debounceSec} (default {@code 0.15 s}) while farther than
 * {@code minGoalDistanceMeters} (default {@code 1.0 m}) from the goal.
 */
public final class CollisionDetector {
  /** Maximum ratio of actual-to-expected speed considered stalled. */
  private final double speedRatioThreshold;

  /** Continuous stall duration required before triggering recovery. */
  private final double debounceSec;

  /** Minimum remaining distance to goal for stall detection to stay armed. */
  private final double minGoalDistanceMeters;

  /** Minimum commanded speed below which stall detection is suppressed. */
  private final double minExpectedSpeedMps;

  /** Accumulated continuous stall duration in seconds. */
  private double stalledDurationSec = 0.0;

  /** Previous update timestamp in seconds, or negative before the first update. */
  private double lastTimestampSec = -1.0;

  /** Creates a detector with FRC competition defaults ({@code 0.45}, {@code 0.15 s}, {@code 1.0 m}, {@code 0.75 m/s}). */
  public CollisionDetector() {
    this(0.45, 0.15, 1.0, 0.75);
  }

  /**
   * Creates a trajectory-stall detector with explicit thresholds.
   *
   * @param speedRatioThreshold ratio in {@code (0, 1]} below which motion counts as stalled
   * @param debounceSec positive debounce window in seconds
   * @param minGoalDistanceMeters non-negative goal proximity suppression distance in meters
   * @param minExpectedSpeedMps positive minimum commanded speed in m/s
   */
  public CollisionDetector(
      double speedRatioThreshold,
      double debounceSec,
      double minGoalDistanceMeters,
      double minExpectedSpeedMps) {
    if (!Double.isFinite(speedRatioThreshold)
        || speedRatioThreshold <= 0.0
        || speedRatioThreshold > 1.0) {
      throw new IllegalArgumentException("speedRatioThreshold must be finite and in (0, 1].");
    }
    if (!Double.isFinite(debounceSec) || debounceSec <= 0.0) {
      throw new IllegalArgumentException("debounceSec must be finite and positive.");
    }
    if (!Double.isFinite(minGoalDistanceMeters) || minGoalDistanceMeters < 0.0) {
      throw new IllegalArgumentException("minGoalDistanceMeters must be finite and non-negative.");
    }
    if (!Double.isFinite(minExpectedSpeedMps) || minExpectedSpeedMps <= 0.0) {
      throw new IllegalArgumentException("minExpectedSpeedMps must be finite and positive.");
    }
    this.speedRatioThreshold = speedRatioThreshold;
    this.debounceSec = debounceSec;
    this.minGoalDistanceMeters = minGoalDistanceMeters;
    this.minExpectedSpeedMps = minExpectedSpeedMps;
  }

  /**
   * Updates the detector for the current tick and returns {@code true} if autonomous collision
   * recovery should trigger.
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

  /** Resets accumulated stall timers between autonomous segments. */
  public void reset() {
    stalledDurationSec = 0.0;
    lastTimestampSec = -1.0;
  }
}
