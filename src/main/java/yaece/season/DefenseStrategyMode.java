// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

/** Defensive targeting strategy selectable by the driver or automated by YAECE. */
public enum DefenseStrategyMode {
  /** Dynamically intercept the opponent with the highest immediate scoring threat. */
  HIGHEST_THREAT,
  /** Patrol and block opponents entering designated high-value defense corridors/zones. */
  ZONE_DEFENSE,
  /**
   * Automatically choose between {@link #HIGHEST_THREAT} and {@link #ZONE_DEFENSE} based on the
   * live/projected alliance score margin and {@link DefenseConfigSpec#defaultPreferredMode()}.
   */
  AUTO_ADAPTIVE
}
