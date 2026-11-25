// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.bridges;

/** Controls NetworkTables 4 telemetry volume published by {@link AdvantageScopeTelemetryBridge}. */
public enum TelemetryVerbosity {
  /** Publishes nothing (zero NT4 overhead). */
  OFF,
  /** Publishes only active intent, target ID, recommended pose, and pin-backoff alert. */
  COMPETITION_MINIMAL,
  /** Publishes full 3D {@code Pose3d[]} Struct arrays for AdvantageScope / YACSim visualization. */
  FULL_ADVANTAGESCOPE
}
