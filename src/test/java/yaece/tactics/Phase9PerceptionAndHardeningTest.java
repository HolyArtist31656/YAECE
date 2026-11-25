// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import yaece.commands3.CommandBindingGuard;
import yaece.util.InitWallTimeWatchdog;
import yaece.world.SlotOccupancyGrid;

class Phase9PerceptionAndHardeningTest {

  @Test
  void slotOccupancyGridSaturatesAtMaxAndClearsInNineMisses() {
    SlotOccupancyGrid grid = new SlotOccupancyGrid();
    assertEquals(0.50, grid.getProbability("hub_slot_1"), 1e-6);

    // Saturate occupancy with repeated hits -> clamps at 0.97
    for (int i = 0; i < 10; i++) {
      grid.recordHit("hub_slot_1", i * 0.02);
    }
    assertEquals(0.97, grid.getProbability("hub_slot_1"), 1e-3);
    assertTrue(grid.isOccupied("hub_slot_1"));
    assertTrue(grid.getOccupiedSlotIds().contains("hub_slot_1"));

    // ~9 consecutive misses must flip saturated slot back below 0.50 (and <= freeThreshold soon)
    for (int i = 0; i < 9; i++) {
      grid.recordMiss("hub_slot_1", 1.0 + i * 0.02);
    }
    assertTrue(
        grid.getProbability("hub_slot_1") < 0.50, "9 ray misses should clear saturated slot");
    grid.recordMiss("hub_slot_1", 1.20);
    grid.recordMiss("hub_slot_1", 1.22);
    assertTrue(grid.isFree("hub_slot_1"));
  }

  @Test
  void initWallTimeWatchdogUsesNanoTimeAndTracksPhases() throws InterruptedException {
    InitWallTimeWatchdog wd = new InitWallTimeWatchdog(500.0);
    Thread.sleep(5L);
    double p1 = wd.markPhase("loadSeason");
    assertTrue(p1 >= 1.0, "System.nanoTime() must advance even when WPILib Timer is frozen");
    assertFalse(wd.isOverBudget());
    assertTrue(wd.getPhaseMillis().containsKey("loadSeason"));

    InitWallTimeWatchdog tight = new InitWallTimeWatchdog(0.001);
    Thread.sleep(2L);
    assertTrue(tight.isOverBudget());
  }

  @Test
  void commandBindingGuardPreventsDuplicateBindingsAcrossReEnable() {
    CommandBindingGuard guard = new CommandBindingGuard();
    AtomicInteger bindCount = new AtomicInteger(0);

    // Simulating repeated disable -> enable cycles in ForOpMode / RobotModeScope (#9558)
    assertTrue(guard.bindOnce("driver_score_trigger", bindCount::incrementAndGet));
    assertFalse(guard.bindOnce("driver_score_trigger", bindCount::incrementAndGet));
    assertFalse(guard.bindOnce("driver_score_trigger", bindCount::incrementAndGet));
    assertEquals(1, bindCount.get());

    // After explicit onExit scope teardown, re-binding is allowed cleanly
    guard.resetScope();
    assertTrue(guard.bindOnce("driver_score_trigger", bindCount::incrementAndGet));
    assertEquals(2, bindCount.get());
  }
}
