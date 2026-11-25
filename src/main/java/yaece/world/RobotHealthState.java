// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

/**
 * Optional real-time chassis and power health inputs fed into {@link FieldWorldModel} and {@link
 * yaece.tactics.YaeceEngine} from drivetrain odometry, IMU skid/tilt detectors, or battery brownout
 * monitors.
 *
 * @param odometryConfidence normalized pose-estimate trust in {@code [0.0, 1.0]}
 * @param isSkiddingOrTilted true when wheel slip or raised-element tilt degrades pose reliability
 * @param maxSpeedThrottleFactor drivetrain speed scaling factor in {@code [0.25, 1.0]} (e.g. under
 *     low-voltage brownout throttling)
 * @param collisionDetected true when a hard impact or sustained stall was recently detected
 */
public record RobotHealthState(
    double odometryConfidence,
    boolean isSkiddingOrTilted,
    double maxSpeedThrottleFactor,
    boolean collisionDetected) {

  public static final RobotHealthState NOMINAL = new RobotHealthState(1.0, false, 1.0, false);

  public RobotHealthState {
    odometryConfidence = Math.max(0.0, Math.min(1.0, odometryConfidence));
    maxSpeedThrottleFactor = Math.max(0.25, Math.min(1.0, maxSpeedThrottleFactor));
  }
}
