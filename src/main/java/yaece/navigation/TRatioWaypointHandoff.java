package yaece.navigation;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;

/**
 * BLine-style {@code t}-ratio projection waypoint handoff controller preventing laterally bumped
 * robots from reversing back to missed intermediate waypoints.
 *
 * <p>For segment {@code A -> B} of length {@code L} and handoff radius {@code r}, the robot
 * advances to the next waypoint as soon as either:
 *
 * <ul>
 *   <li>its Euclidean distance to {@code B} is within {@code r}, or
 *   <li>its scalar projection ratio along {@code A -> B} satisfies {@code proj >= clamp(1 - r/L, 0,
 *       1)}.
 * </ul>
 */
public final class TRatioWaypointHandoff {

  /**
   * Snapshot of the active waypoint evaluation.
   *
   * @param activeIndex Current target waypoint index in the polyline.
   * @param projectionRatio Scalar projection ratio along the current segment.
   * @param handoffThreshold {@code clamp(1 - r/L, 0, 1)} threshold for the current segment.
   * @param advanced {@code true} if the handoff triggered during this update.
   * @param atFinalWaypoint {@code true} when targeting the terminal goal waypoint.
   */
  public record HandoffState(
      int activeIndex,
      double projectionRatio,
      double handoffThreshold,
      boolean advanced,
      boolean atFinalWaypoint) {}

  private final double handoffRadiusMeters;

  /**
   * Creates a {@code t}-ratio handoff helper.
   *
   * @param handoffRadiusMeters Positive intermediate waypoint handoff radius in meters (typically
   *     {@code 0.25 - 0.50 m}).
   */
  public TRatioWaypointHandoff(double handoffRadiusMeters) {
    if (handoffRadiusMeters <= 0.0 || !Double.isFinite(handoffRadiusMeters)) {
      throw new IllegalArgumentException("handoffRadiusMeters must be positive and finite.");
    }
    this.handoffRadiusMeters = handoffRadiusMeters;
  }

  /**
   * @return Configured handoff radius in meters.
   */
  public double getHandoffRadiusMeters() {
    return handoffRadiusMeters;
  }

  /**
   * Evaluates whether the robot should advance past {@code currentIndex} along {@code waypoints}.
   * Never advances past the terminal waypoint {@code waypoints.size() - 1}.
   *
   * @param waypoints Ordered polyline waypoints (at least 1).
   * @param currentIndex Current target index in {@code [0, waypoints.size() - 1]}.
   * @param robotPosition Current robot X/Y position in meters.
   * @return Updated {@link HandoffState}.
   */
  public HandoffState evaluate(
      List<Translation2d> waypoints, int currentIndex, Translation2d robotPosition) {
    Objects.requireNonNull(waypoints, "waypoints");
    Objects.requireNonNull(robotPosition, "robotPosition");
    if (waypoints.isEmpty()) {
      throw new IllegalArgumentException("waypoints must not be empty.");
    }
    int idx = Math.max(0, Math.min(currentIndex, waypoints.size() - 1));
    int lastIdx = waypoints.size() - 1;
    if (idx >= lastIdx) {
      return new HandoffState(lastIdx, 1.0, 1.0, false, true);
    }

    boolean advanced = false;
    double proj = 0.0;
    double threshold = 1.0;

    while (idx < lastIdx) {
      Translation2d target = waypoints.get(idx);
      Translation2d prev = idx > 0 ? waypoints.get(idx - 1) : robotPosition;
      double dx = target.getX() - prev.getX();
      double dy = target.getY() - prev.getY();
      double segLen = Math.hypot(dx, dy);

      double distToTarget = robotPosition.getDistance(target);
      if (segLen < 1e-6) {
        proj = 1.0;
        threshold = 0.0;
      } else {
        double rx = robotPosition.getX() - prev.getX();
        double ry = robotPosition.getY() - prev.getY();
        proj = (rx * dx + ry * dy) / (segLen * segLen);
        threshold = clamp(1.0 - (handoffRadiusMeters / segLen), 0.0, 1.0);
      }

      if (distToTarget <= handoffRadiusMeters || proj >= threshold) {
        idx++;
        advanced = true;
      } else {
        break;
      }
    }

    return new HandoffState(idx, proj, threshold, advanced, idx == lastIdx);
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
