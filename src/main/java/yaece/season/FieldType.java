// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

/**
 * Official FRC field perimeter manufacturing variant, capturing the dimensional delta between
 * welded (chassis-perimeter) and AndyMark competition fields.
 */
public enum FieldType {
  /** Welded competition field perimeter (e.g., 54 ft 3.2 in x 26 ft 5.7 in). */
  WELDED(16.540, 8.070),

  /** AndyMark competition field perimeter (e.g., 54 ft 2.12 in x 26 ft 4.64 in). */
  ANDYMARK(16.513, 8.043);

  private final double nominalLengthMeters;
  private final double nominalWidthMeters;

  FieldType(double nominalLengthMeters, double nominalWidthMeters) {
    this.nominalLengthMeters = nominalLengthMeters;
    this.nominalWidthMeters = nominalWidthMeters;
  }

  /** Returns the nominal field length (X axis) in meters for this field variant. */
  public double nominalLengthMeters() {
    return nominalLengthMeters;
  }

  /** Returns the nominal field width (Y axis) in meters for this field variant. */
  public double nominalWidthMeters() {
    return nominalWidthMeters;
  }
}
