// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.RobotAffiliation;
import yaece.season.PinRuleSpec;

class PinGuardTrackerTest {

  private static TrackedRobot opponentAt(int id, double x, double y, double vx) {
    return new TrackedRobot(
        id,
        RobotAffiliation.OPPONENT,
        new Pose3d(new Translation3d(x, y, 0.0), new Rotation3d()),
        vx,
        0.0,
        0.45,
        0.9,
        0.05,
        1.0,
        false,
        10.0);
  }

  @Test
  void enforces3SecondPinRuleWithPauseResumeAndFullReset() {
    PinGuardTracker guard = new PinGuardTracker(PinRuleSpec.FRC_DEFAULT, 0.45);
    Translation2d ego = new Translation2d(2.0, 2.0);

    // Contact stalled opponent (center dist = 0.95m -> bumper gap = 0.05m <= 0.22m) for 2.3s
    Map<Integer, OpponentPinState> st = Map.of();
    for (int i = 0; i < 23; i++) {
      st = guard.update(ego, List.of(opponentAt(1, 2.95, 2.0, 0.05)), 0.1);
    }
    OpponentPinState pinned = st.get(1);
    assertEquals(PinStatus.PINNING_ACTIVE, pinned.status());
    assertTrue(pinned.backoffRequired());
    assertEquals(2.3, pinned.activePinSeconds(), 1e-6);

    // Back off past 1.83m bumper gap (center dist = 3.0m -> gap = 2.10m) for 1.5s ->
    // COOLDOWN_PAUSED
    for (int i = 0; i < 15; i++) {
      st = guard.update(ego, List.of(opponentAt(1, 5.0, 2.0, 0.0)), 0.1);
    }
    assertEquals(PinStatus.COOLDOWN_PAUSED, st.get(1).status());
    assertEquals(2.3, st.get(1).activePinSeconds(), 1e-6);

    // Re-enter too early (gap = 1.0m < 1.83m) -> resumes paused count from 2.3s!
    st = guard.update(ego, List.of(opponentAt(1, 3.9, 2.0, 0.0)), 0.2);
    assertEquals(PinStatus.PINNING_ACTIVE, st.get(1).status());
    assertEquals(2.5, st.get(1).activePinSeconds(), 1e-6);

    // Now back off past 1.83m for full 3.3s -> resets cleanly to FREE (0.0s)
    for (int i = 0; i < 33; i++) {
      st = guard.update(ego, List.of(opponentAt(1, 5.2, 2.0, 0.0)), 0.1);
    }
    assertEquals(PinStatus.FREE, st.get(1).status());
    assertEquals(0.0, st.get(1).activePinSeconds(), 1e-6);
    assertFalse(st.get(1).backoffRequired());
  }
}
