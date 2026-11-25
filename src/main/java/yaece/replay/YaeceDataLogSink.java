// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.replay;

import java.util.Objects;
import org.wpilib.datalog.BooleanLogEntry;
import org.wpilib.datalog.DataLog;
import org.wpilib.datalog.DoubleLogEntry;
import org.wpilib.datalog.StringLogEntry;
import org.wpilib.datalog.StructLogEntry;
import org.wpilib.math.geometry.Pose3d;
import yaece.tactics.TacticalRecommendation;
import yaece.world.RobotHealthState;
import yaece.world.WorldSnapshot;

/**
 * Zero-dependency WPILib {@link DataLog} ({@code .wpilog}) telemetry sink for competition matches
 * where high-rate NetworkTables 4 publishing is disabled to conserve SystemCore bandwidth.
 */
public final class YaeceDataLogSink {
  private final StructLogEntry<Pose3d> egoPoseEntry;
  private final StructLogEntry<Pose3d> targetPoseEntry;
  private final StringLogEntry intentEntry;
  private final StringLogEntry targetIdEntry;
  private final DoubleLogEntry utilityEntry;
  private final DoubleLogEntry odometryConfidenceEntry;
  private final DoubleLogEntry throttleFactorEntry;
  private final BooleanLogEntry pinBackoffEntry;

  public YaeceDataLogSink(DataLog log) {
    this(log, "yaece");
  }

  public YaeceDataLogSink(DataLog log, String prefix) {
    Objects.requireNonNull(log, "log must not be null");
    String base = (prefix == null || prefix.isBlank()) ? "yaece" : prefix;
    this.egoPoseEntry = StructLogEntry.create(log, base + "/egoPose", Pose3d.struct);
    this.targetPoseEntry = StructLogEntry.create(log, base + "/recommendedTarget", Pose3d.struct);
    this.intentEntry = new StringLogEntry(log, base + "/activeIntent");
    this.targetIdEntry = new StringLogEntry(log, base + "/activeTargetId");
    this.utilityEntry = new DoubleLogEntry(log, base + "/utilityScore");
    this.odometryConfidenceEntry = new DoubleLogEntry(log, base + "/odometryConfidence");
    this.throttleFactorEntry = new DoubleLogEntry(log, base + "/maxSpeedThrottleFactor");
    this.pinBackoffEntry = new BooleanLogEntry(log, base + "/pinBackoffRequired");
  }

  /** Appends one 50 Hz tactical snapshot directly to the underlying WPILib {@link DataLog}. */
  public void logTick(
      WorldSnapshot world, TacticalRecommendation rec, RobotHealthState health, long timestampUs) {
    Objects.requireNonNull(world, "world must not be null");
    Objects.requireNonNull(rec, "rec must not be null");
    RobotHealthState h = health == null ? RobotHealthState.NOMINAL : health;
    egoPoseEntry.append(world.egoPose3d(), timestampUs);
    targetPoseEntry.append(rec.targetPose3d(), timestampUs);
    intentEntry.append(rec.intent().name(), timestampUs);
    targetIdEntry.append(rec.targetId(), timestampUs);
    utilityEntry.append(rec.utilityScore(), timestampUs);
    odometryConfidenceEntry.append(h.odometryConfidence(), timestampUs);
    throttleFactorEntry.append(h.maxSpeedThrottleFactor(), timestampUs);
    pinBackoffEntry.append(world.anyPinBackoffRequired(), timestampUs);
  }
}
