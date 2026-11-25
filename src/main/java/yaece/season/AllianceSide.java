// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

/** Alliance side on an FRC playing field. */
public enum AllianceSide {
  /** Blue alliance (canonical WPILib origin). */
  BLUE,
  /** Red alliance (mirrored from Blue according to {@link FieldMirrorMode}). */
  RED;

  /** Returns the opposing alliance side. */
  public AllianceSide opposite() {
    return this == BLUE ? RED : BLUE;
  }
}
