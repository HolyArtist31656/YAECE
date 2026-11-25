// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;
import yaece.perception.RobotAffiliation;

/**
 * Immutable snapshot of a tracked non-ego robot (ally, opponent, or unclassified obstacle) with
 * 2-mode IMM motion estimates and occlusion-aware uncertainty.
 */
public record TrackedRobot(
    int trackId,
    RobotAffiliation affiliation,
    Pose3d pose3d,
    double vxMps,
    double vyMps,
    double estimatedRadiusMeters,
    double stationaryProbability,
    double positionUncertaintyMeters,
    double existenceConfidence,
    boolean occluded,
    double lastSeenTimestampSec) {

  /** Returns the planar speed magnitude in m/s. */
  public double speedMps() {
    return Math.hypot(vxMps, vyMps);
  }

  /** Projects this robot's 2D field position forward by {@code dtSec} seconds. */
  public Translation2d predictPositionAt(double dtSec) {
    double dt = Math.max(0.0, dtSec);
    return new Translation2d(pose3d.getX() + vxMps * dt, pose3d.getY() + vyMps * dt);
  }

  /**
   * Returns the expanded 1-sigma position uncertainty radius (meters) at lookahead {@code dtSec},
   * accounting for human driver acceleration uncertainty (~3.0 m/s^2).
   */
  public double predictUncertaintyRadiusAt(double dtSec) {
    double dt = Math.max(0.0, dtSec);
    double movingFactor = 1.0 - 0.5 * stationaryProbability;
    return positionUncertaintyMeters + movingFactor * (0.35 * dt + 0.45 * dt * dt);
  }
}
