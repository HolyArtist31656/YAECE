// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.commands3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import yaece.util.InitWallTimeWatchdog;

/** Verifies YAECE Commands v3 compatibility and lifecycle guards on WPILib 2027.0.0-alpha-8. */
class YaeceAlpha8Commands3Test {
  private static final int REENABLE_CYCLES = 5;
  private static final double INIT_BUDGET_MS = 25.0;
  private static final double WATCHDOG_TIMEOUT_SEC = 1.5;

  @Test
  void commandBindingGuardPreventsDuplicateTriggerRegistrationAcrossReenableCycles() {
    CommandBindingGuard guard = new CommandBindingGuard();
    AtomicInteger bindCount = new AtomicInteger(0);

    for (int i = 0; i < REENABLE_CYCLES; i++) {
      boolean bound = guard.bindOnce("teleop.encounter.trigger", bindCount::incrementAndGet);
      if (i == 0) {
        assertTrue(bound);
      } else {
        assertFalse(bound);
      }
    }
    assertEquals(1, bindCount.get());
    assertTrue(guard.isBound("teleop.encounter.trigger"));

    guard.resetScope();
    assertFalse(guard.isBound("teleop.encounter.trigger"));
  }

  @Test
  void possessionWatchdogWrapsCommandsV3CoroutineCleanlyOnAlpha8() {
    Command dummyScore = Command.noRequirements(co -> co.yield()).named("Dummy Score");
    Command watched =
        YaeceCommandFactory.withPossessionWatchdog(
            dummyScore, () -> true, () -> 0.0, WATCHDOG_TIMEOUT_SEC);
    assertNotNull(watched);
    assertEquals("YAECE Possession Watchdog", watched.name());
  }

  @Test
  void initWallTimeWatchdogUsesMonotonicSystemNanoTimeUnderAlpha8() {
    InitWallTimeWatchdog watchdog = new InitWallTimeWatchdog(INIT_BUDGET_MS);
    double phaseMs = watchdog.markPhase("alpha8-init");
    double totalMs = watchdog.getTotalElapsedMillis();
    assertTrue(Double.isFinite(phaseMs) && phaseMs >= 0.0);
    assertTrue(Double.isFinite(totalMs) && totalMs >= phaseMs);
  }
}
