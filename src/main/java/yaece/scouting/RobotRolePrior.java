// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.scouting;

/**
 * Pre-match scouting role classification for an opponent robot, used to weight defensive target
 * selection and encounter arbitration when multiple opponents are simultaneously active.
 */
public enum RobotRolePrior {
  /** High-volume cycle scorer that should be prioritized for defensive interception. */
  PRIMARY_SCORER(1.45, 4.5),

  /** Moderate scorer with standard threat priority. */
  SECONDARY_SCORER(1.0, 7.0),

  /** Shuttle or feeder robot supplying game pieces to teammates. */
  FEEDER_PASSER(1.15, 5.5),

  /** Dedicated defense robot that is typically low value to counter-defend. */
  DEFENDER(0.45, 18.0),

  /** Robot whose primary point contribution comes during endgame climb or bonus tasks. */
  ENDGAME_SPECIALIST(0.85, 12.0),

  /** Unclassified robot with neutral threat weighting. */
  UNKNOWN(1.0, 8.0);

  private final double baseThreatMultiplier;
  private final double nominalCycleTimeSec;

  RobotRolePrior(double baseThreatMultiplier, double nominalCycleTimeSec) {
    this.baseThreatMultiplier = baseThreatMultiplier;
    this.nominalCycleTimeSec = nominalCycleTimeSec;
  }

  /** Returns the baseline threat multiplier applied in defensive target scoring. */
  public double baseThreatMultiplier() {
    return baseThreatMultiplier;
  }

  /** Returns the nominal cycle time estimate (seconds) for this archetype. */
  public double nominalCycleTimeSec() {
    return nominalCycleTimeSec;
  }
}
