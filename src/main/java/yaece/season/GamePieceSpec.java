// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

import java.util.Objects;

/** Immutable physical, perception, and inventory specification for one FRC game-piece type. */
public record GamePieceSpec(
    String id,
    int visionClassId,
    double lengthMeters,
    double widthMeters,
    double heightMeters,
    double restHeightMeters,
    GamePieceSymmetry symmetry,
    int maxHeldSimultaneously,
    boolean requireOrientationForIntake,
    double maxIntakeYawErrorDeg,
    double maxIntakeTiltErrorDeg) {

  public GamePieceSpec {
    if (id == null || id.isBlank()) {
      throw new IllegalArgumentException("GamePieceSpec.id must be non-blank");
    }
    if (visionClassId < 0) {
      throw new IllegalArgumentException("GamePieceSpec.visionClassId must be >= 0");
    }
    if (!Double.isFinite(lengthMeters) || lengthMeters <= 0.0) {
      throw new IllegalArgumentException("GamePieceSpec.lengthMeters must be finite and > 0");
    }
    if (!Double.isFinite(widthMeters) || widthMeters <= 0.0) {
      throw new IllegalArgumentException("GamePieceSpec.widthMeters must be finite and > 0");
    }
    if (!Double.isFinite(heightMeters) || heightMeters <= 0.0) {
      throw new IllegalArgumentException("GamePieceSpec.heightMeters must be finite and > 0");
    }
    if (!Double.isFinite(restHeightMeters) || restHeightMeters < 0.0) {
      throw new IllegalArgumentException("GamePieceSpec.restHeightMeters must be finite and >= 0");
    }
    Objects.requireNonNull(symmetry, "GamePieceSpec.symmetry must not be null");
    if (maxHeldSimultaneously < 1) {
      throw new IllegalArgumentException("GamePieceSpec.maxHeldSimultaneously must be >= 1");
    }
    if (!Double.isFinite(maxIntakeYawErrorDeg)
        || maxIntakeYawErrorDeg < 0.0
        || maxIntakeYawErrorDeg > 180.0) {
      throw new IllegalArgumentException(
          "GamePieceSpec.maxIntakeYawErrorDeg must be within [0, 180]");
    }
    if (!Double.isFinite(maxIntakeTiltErrorDeg)
        || maxIntakeTiltErrorDeg < 0.0
        || maxIntakeTiltErrorDeg > 180.0) {
      throw new IllegalArgumentException(
          "GamePieceSpec.maxIntakeTiltErrorDeg must be within [0, 180]");
    }
  }

  /** Returns the nominal bounding radius in the horizontal XY plane in meters. */
  public double boundingRadiusXyMeters() {
    return 0.5 * Math.hypot(lengthMeters, widthMeters);
  }
}
