package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import yaece.commands3.CommandBindingGuard;
import yaece.util.InitWallTimeWatchdog;
import yaece.world.SlotOccupancyGrid;
import yaece.world.VisionFusionPolicy;

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
  void visionFusionPolicyHandlesPreEnableGyroZeroAndEnabledGating() {
    VisionFusionPolicy policy = new VisionFusionPolicy();

    // Pre-enable Phase 1: >=2 tags and >=60% inliers triggers hardResetPose + low yaw std-dev
    var phase1 = policy.evaluate(false, 2, 2.5, 0.65, 5.0, 0.0, false);
    assertTrue(phase1.accepted());
    assertTrue(phase1.hardResetPose());
    assertTrue(phase1.thetaStdDevRad() < 0.1);

    // Enabled Phase 2: rejects >360 deg/s angular velocity
    var spinReject = policy.evaluate(true, 3, 2.0, 0.95, 0.1, 420.0, true);
    assertFalse(spinReject.accepted());
    assertEquals("high_angular_rate", spinReject.reason());

    // Enabled Phase 2: rejects >2.0 m polycarbonate reflection jump
    var jumpReject = policy.evaluate(true, 3, 2.0, 0.95, 2.6, 10.0, true);
    assertFalse(jumpReject.accepted());
    assertEquals("pose_jump_rejected", jumpReject.reason());

    // Enabled Phase 2: 3+ tag shooter camera gets 0.35x XY trust boost and high yaw std-dev
    var normal = policy.evaluate(true, 3, 3.0, 0.90, 0.15, 25.0, false);
    var shooter = policy.evaluate(true, 3, 3.0, 0.90, 0.15, 25.0, true);
    assertTrue(normal.accepted() && shooter.accepted());
    assertFalse(shooter.hardResetPose());
    assertEquals(normal.xyStdDevMeters() * 0.35, shooter.xyStdDevMeters(), 1e-6);
    assertEquals(VisionFusionPolicy.HIGH_YAW_STD_DEV_RAD, shooter.thetaStdDevRad(), 1e-6);
  }

  @Test
  void visionFusionPolicyRejectsZeroTagsAndFarSingleTag() {
    VisionFusionPolicy policy = new VisionFusionPolicy();
    assertFalse(policy.evaluate(true, 0, 2.0, 1.0, 0.0, 0.0, false).accepted());
    assertFalse(policy.evaluate(true, 1, 4.5, 0.95, 0.1, 0.0, false).accepted());
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
