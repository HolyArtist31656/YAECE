// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.perception;

/**
 * Bumper-height 2D LiDAR arc cluster expressed in the ego robot's XY frame (+X forward, +Y left).
 *
 * @param centerXMeters cluster centroid X in robot frame (meters)
 * @param centerYMeters cluster centroid Y in robot frame (meters)
 * @param clusterWidthMeters chord width of the LiDAR arc (meters)
 * @param confidence quality/density confidence in [0, 1]
 * @param timestampSec scan capture timestamp in seconds
 */
public record LidarCluster(
    double centerXMeters,
    double centerYMeters,
    double clusterWidthMeters,
    double confidence,
    double timestampSec) {

  public LidarCluster {
    if (!Double.isFinite(centerXMeters) || !Double.isFinite(centerYMeters)) {
      throw new IllegalArgumentException("LidarCluster center coordinates must be finite");
    }
    if (!Double.isFinite(clusterWidthMeters) || clusterWidthMeters <= 0.0) {
      throw new IllegalArgumentException("LidarCluster width must be finite and > 0");
    }
    confidence = Math.max(0.0, Math.min(1.0, Double.isFinite(confidence) ? confidence : 0.0));
    if (!Double.isFinite(timestampSec) || timestampSec < 0.0) {
      throw new IllegalArgumentException("timestampSec must be finite and >= 0");
    }
  }

  /** Returns the polar bearing angle in radians in the robot XY frame. */
  public double bearingRad() {
    return Math.atan2(centerYMeters, centerXMeters);
  }

  /** Returns the planar range from robot center in meters. */
  public double rangeMeters() {
    return Math.hypot(centerXMeters, centerYMeters);
  }
}
