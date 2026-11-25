package yaece.control;

import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;

/**
 * COMPASS-style topological passing-side commitment filter with cost hysteresis {@code (1 + h)} and
 * minimum dwell time to eliminate head-on passing oscillation and livelock against oncoming robots.
 */
public final class TopologicalPassingCommitment {

  /** Committed topological passing side relative to the ego-to-goal heading vector. */
  public enum PassingSide {
    NONE,
    LEFT,
    RIGHT
  }

  private final double hysteresisRatio;
  private final double minDwellSeconds;
  private PassingSide committedSide = PassingSide.NONE;
  private double lastSwitchTimeSec = Double.NEGATIVE_INFINITY;

  /**
   * Creates a topological passing commitment filter.
   *
   * @param hysteresisRatio Fractional cost advantage required to flip passing sides (e.g., {@code
   *     0.25} requires a 25% lower cost to flip).
   * @param minDwellSeconds Minimum time in seconds to hold a committed side before flipping (e.g.,
   *     {@code 0.40 s}).
   */
  public TopologicalPassingCommitment(double hysteresisRatio, double minDwellSeconds) {
    if (hysteresisRatio < 0.0 || !Double.isFinite(hysteresisRatio)) {
      throw new IllegalArgumentException("hysteresisRatio must be non-negative and finite.");
    }
    if (minDwellSeconds < 0.0 || !Double.isFinite(minDwellSeconds)) {
      throw new IllegalArgumentException("minDwellSeconds must be non-negative and finite.");
    }
    this.hysteresisRatio = hysteresisRatio;
    this.minDwellSeconds = minDwellSeconds;
  }

  /**
   * @return Currently committed {@link PassingSide}.
   */
  public PassingSide getCommittedSide() {
    return committedSide;
  }

  /** Resets any active passing commitment. */
  public void reset() {
    committedSide = PassingSide.NONE;
    lastSwitchTimeSec = Double.NEGATIVE_INFINITY;
  }

  /**
   * Evaluates left vs. right passing costs against an obstacle blocking the ego-to-goal line and
   * updates the committed passing side with hysteresis and dwell-time enforcement.
   *
   * @param nowSec Monotonic timestamp in seconds.
   * @param egoPos Ego X/Y position in meters.
   * @param goalPos Goal X/Y position in meters.
   * @param obstaclePos Blocking obstacle/robot X/Y position in meters.
   * @param leftCost Lower-is-better traversal cost for passing on the left.
   * @param rightCost Lower-is-better traversal cost for passing on the right.
   * @return Committed {@link PassingSide}.
   */
  public PassingSide update(
      double nowSec,
      Translation2d egoPos,
      Translation2d goalPos,
      Translation2d obstaclePos,
      double leftCost,
      double rightCost) {
    Objects.requireNonNull(egoPos, "egoPos");
    Objects.requireNonNull(goalPos, "goalPos");
    Objects.requireNonNull(obstaclePos, "obstaclePos");

    // Check if obstacle is already behind ego relative to ego->goal vector
    double gx = goalPos.getX() - egoPos.getX();
    double gy = goalPos.getY() - egoPos.getY();
    double ox = obstaclePos.getX() - egoPos.getX();
    double oy = obstaclePos.getY() - egoPos.getY();
    if (gx * ox + gy * oy <= 0.0) {
      committedSide = PassingSide.NONE;
      return committedSide;
    }

    PassingSide preferred = leftCost <= rightCost ? PassingSide.LEFT : PassingSide.RIGHT;
    if (committedSide == PassingSide.NONE) {
      committedSide = preferred;
      lastSwitchTimeSec = nowSec;
      return committedSide;
    }

    if (preferred == committedSide) {
      return committedSide;
    }

    // Enforce minimum dwell time
    if ((nowSec - lastSwitchTimeSec) < minDwellSeconds) {
      return committedSide;
    }

    // Enforce multiplicative (1 + h) hysteresis before flipping sides
    double currentCost = committedSide == PassingSide.LEFT ? leftCost : rightCost;
    double candidateCost = preferred == PassingSide.LEFT ? leftCost : rightCost;
    if (currentCost > candidateCost * (1.0 + hysteresisRatio)) {
      committedSide = preferred;
      lastSwitchTimeSec = nowSec;
    }
    return committedSide;
  }
}
