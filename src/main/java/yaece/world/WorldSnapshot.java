// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose3d;
import yaece.perception.RobotAffiliation;

/** Immutable, thread-safe snapshot of the entire 3D FRC field world model at a single timestamp. */
public record WorldSnapshot(
    double timestampSec,
    Pose3d egoPose3d,
    double egoVxMps,
    double egoVyMps,
    List<TrackedGamePiece> gamePieces,
    List<TrackedRobot> robots,
    Map<Integer, OpponentPinState> pinStates) {

  /** Empty baseline snapshot at timestamp {@code 0.0}. */
  public static final WorldSnapshot EMPTY =
      new WorldSnapshot(0.0, new Pose3d(), 0.0, 0.0, List.of(), List.of(), Map.of());

  /** Returns all tracked opposing alliance robots without stream pipeline allocation. */
  public List<TrackedRobot> opponents() {
    return filterByAffiliation(RobotAffiliation.OPPONENT);
  }

  /** Returns all tracked alliance partner robots without stream pipeline allocation. */
  public List<TrackedRobot> allies() {
    return filterByAffiliation(RobotAffiliation.ALLY);
  }

  private List<TrackedRobot> filterByAffiliation(RobotAffiliation affiliation) {
    if (robots.isEmpty()) {
      return List.of();
    }
    List<TrackedRobot> matched = new ArrayList<>(robots.size());
    for (int i = 0; i < robots.size(); i++) {
      TrackedRobot robot = robots.get(i);
      if (robot.affiliation() == affiliation) {
        matched.add(robot);
      }
    }
    return matched.isEmpty() ? List.of() : List.copyOf(matched);
  }

  /** Returns true if any opponent pin timer has reached the proactive backoff threshold. */
  public boolean anyPinBackoffRequired() {
    for (OpponentPinState st : pinStates.values()) {
      if (st.backoffRequired()) {
        return true;
      }
    }
    return false;
  }
}
