// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.perception;

import java.util.Objects;

/**
 * Multi-camera bumper detection bearing in the ego robot's XY frame, used to stamp {@link
 * RobotAffiliation#ALLY} or {@link RobotAffiliation#OPPONENT} onto 2D LiDAR range clusters (or
 * provide camera-only range fallback).
 *
 * @param bearingRad horizontal bearing in robot frame (radians, +CCW / left)
 * @param affiliation detected bumper alliance affiliation
 * @param confidence detector confidence in [0, 1]
 * @param estimatedRangeMeters optional monocular range estimate in meters, or {@code <= 0} if
 *     bearing-only
 * @param timestampSec capture timestamp in seconds
 */
public record CameraBumperBearing(
    double bearingRad,
    RobotAffiliation affiliation,
    double confidence,
    double estimatedRangeMeters,
    double timestampSec) {

  public CameraBumperBearing {
    if (!Double.isFinite(bearingRad)) {
      throw new IllegalArgumentException("bearingRad must be finite");
    }
    Objects.requireNonNull(affiliation, "affiliation must not be null");
    confidence = Math.max(0.0, Math.min(1.0, Double.isFinite(confidence) ? confidence : 0.0));
    if (!Double.isFinite(estimatedRangeMeters)) {
      throw new IllegalArgumentException("estimatedRangeMeters must be finite");
    }
    if (!Double.isFinite(timestampSec) || timestampSec < 0.0) {
      throw new IllegalArgumentException("timestampSec must be finite and >= 0");
    }
  }
}
