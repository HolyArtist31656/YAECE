// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

/** Per-opponent FRC pinning state tracked by {@link PinGuardTracker}. */
public enum PinStatus {
  /** No active pin count against this opponent. */
  FREE,
  /** Ego robot is actively pinning this opponent; timer is counting toward the 3.0 s limit. */
  PINNING_ACTIVE,
  /**
   * Ego robot has separated by >= 1.83 m (6 ft); pin timer is paused and waiting for the 3.2 s
   * cooldown hold to complete before resetting to {@link #FREE}.
   */
  COOLDOWN_PAUSED
}
