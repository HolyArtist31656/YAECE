// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.bridges;

import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import yaece.tactics.YaeceEngine;

/**
 * Zero-dependency QuestNav adapter converting Quest headset {@code Pose3d} frames into robot-center
 * field poses via {@code robotToQuest.inverse()}, with automatic odometry fallback if QuestNav
 * reports tracking loss.
 */
public final class QuestNavAdapter {
  private final Transform3d questToRobot;
  private final Supplier<Pose3d> odometryFallback;

  /**
   * @param robotToQuest mounting transform from robot center to Quest headset
   * @param odometryFallback supplier returning drivetrain odometry {@link Pose3d} when QuestNav is
   *     disconnected or not tracking
   */
  public QuestNavAdapter(Transform3d robotToQuest, Supplier<Pose3d> odometryFallback) {
    Objects.requireNonNull(robotToQuest, "robotToQuest must not be null");
    this.questToRobot = robotToQuest.inverse();
    this.odometryFallback =
        Objects.requireNonNull(odometryFallback, "odometryFallback must not be null");
  }

  /**
   * Transforms a QuestNav frame into robot-center coordinates (or falls back to odometry if {@code
   * isTracking} is false) and feeds {@link YaeceEngine#updateEgoPose(Pose3d, double)}.
   */
  public Pose3d feedFrame(
      YaeceEngine engine, Pose3d questPose3d, double dataTimestampSec, boolean isTracking) {
    Objects.requireNonNull(engine, "engine must not be null");
    Pose3d robotPose =
        (isTracking && questPose3d != null)
            ? questPose3d.plus(questToRobot)
            : odometryFallback.get();
    engine.updateEgoPose(robotPose, dataTimestampSec);
    return robotPose;
  }
}
