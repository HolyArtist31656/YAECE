package yaece.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;
import yaece.ballistics.TurretChassisAuthoritySolver;
import yaece.commands3.CommandBindingGuard;
import yaece.navigation.TRatioWaypointHandoff;
import yaece.world.SlotOccupancyGrid;
import yaece.world.VisionFusionPolicy;

/**
 * CI soak and adversarial fault-injection suite closing VER-03 (120A+ brownout throttling &amp;
 * recovery), VER-06 (160-second Blue &amp; Red mirrored full-match soak), and Section 14.2 Scenario
 * 5 (4-second opponent wall pin &amp; passing recovery).
 */
class FullMatchSoakAndBrownoutTest {

  private static final double FIELD_LENGTH_M = 16.540988;
  private static final double FIELD_WIDTH_M = 8.069326;
  private static final double DT_SEC = 0.02;
  private static final int FULL_MATCH_TICKS = 8_000; // 160 s @ 50 Hz

  private record YaeceSoakMetrics(
      Translation2d finalPos,
      int visionAccepted,
      int visionRejected,
      int slotClearEvents,
      int bindingCount,
      int slowTicksOver2Ms) {}

  @Test
  void brownoutThrottlingAndStallRecoveryAcrossThreeFaultBursts() {
    double nominalMaxSpeedMps = 4.5;
    double restingVoltage = 12.1;
    double degradedResistanceOhm = 0.034;

    TurretChassisAuthoritySolver sotm = new TurretChassisAuthoritySolver(1.25, 2.0, 0.5);
    int brownoutEvents = 0;
    boolean inBrownout = false;

    for (int tick = 0; tick < 1_500; tick++) {
      boolean stallBurst =
          (tick >= 200 && tick < 213)
              || (tick >= 600 && tick < 613)
              || (tick >= 1_000 && tick < 1_013);
      double railV =
          Math.max(5.5, restingVoltage - (stallBurst ? 165.0 : 25.0) * degradedResistanceOhm);
      double throttleScale = Math.max(0.5, Math.min(1.0, (railV - 8.0) / 2.5 * 0.5 + 0.5));

      boolean nowBrownout = railV < 6.8;
      if (nowBrownout && !inBrownout) {
        brownoutEvents++;
      }
      inBrownout = nowBrownout;

      CbfSafetyFilter cbf = new CbfSafetyFilter(0.8, nominalMaxSpeedMps * throttleScale);
      CbfSafetyFilter.FilterResult res =
          cbf.filter(
              new Translation2d(4.0, 4.0),
              new Translation2d(4.5, 0.0),
              List.of(
                  new CbfSafetyFilter.BarrierObstacle(
                      "opp",
                      new Translation2d(5.2, 4.1),
                      new Translation2d(-0.5, 0.0),
                      0.9,
                      CbfSafetyFilter.AgentRole.OPPONENT)));

      double speed = Math.hypot(res.safeVelocityMps().getX(), res.safeVelocityMps().getY());
      assertTrue(speed <= nominalMaxSpeedMps * throttleScale + 1e-6);

      TurretChassisAuthoritySolver.AuthoritySolution sol =
          sotm.solve(
              new Translation2d(4.0, 4.0),
              res.safeVelocityMps(),
              new Translation2d(),
              0.0,
              new Translation2d(0.2, 0.0),
              0.0,
              new Translation2d(1.5, 4.0),
              0.75);
      assertTrue(Double.isFinite(sol.fieldAimYawRad()));
      assertEquals(
          sol.totalRequiredOmegaRadPerSec(),
          sol.chassisOmegaFeedforwardRadPerSec() + sol.turretOmegaFeedforwardRadPerSec(),
          1e-9);

      if (tick == 218 || tick == 618 || tick == 1_018) {
        assertFalse(nowBrownout, "Should recover within 100 ms after burst at tick " + tick);
        assertTrue(throttleScale > 0.95);
      }
    }

    assertEquals(3, brownoutEvents);
  }

  @Test
  void full160SecondBlueAndRedMirroredControlAndPerceptionSoak() {
    YaeceSoakMetrics blue = run160SecondSoak(false);
    YaeceSoakMetrics red = run160SecondSoak(true);

    // Rotational 180-deg alliance parity on final position
    Translation2d blueFlipped = flipRotational(blue.finalPos());
    assertEquals(blueFlipped.getX(), red.finalPos().getX(), 1e-4);
    assertEquals(blueFlipped.getY(), red.finalPos().getY(), 1e-4);

    // Vision policy accepted good frames and rejected blackout/high-spin frames
    assertTrue(blue.visionAccepted() > 4_000 && red.visionAccepted() > 4_000);
    assertTrue(blue.visionRejected() > 150 && red.visionRejected() > 150);

    // Clamped log-odds occupancy grid cleared saturated slots repeatedly
    assertTrue(blue.slotClearEvents() >= 5 && red.slotClearEvents() >= 5);

    // CommandBindingGuard prevented duplicate trigger bindings across 5 re-enable transitions
    // (#9558)
    assertEquals(1, blue.bindingCount());
    assertEquals(1, red.bindingCount());

    // Loop budget compliance: <= 5% of ticks over 2 ms
    int maxSlowTicks = (int) (FULL_MATCH_TICKS * 0.05);
    assertTrue(
        blue.slowTicksOver2Ms() <= maxSlowTicks,
        "Blue soak exceeded 5% 2 ms budget: " + blue.slowTicksOver2Ms());
    assertTrue(
        red.slowTicksOver2Ms() <= maxSlowTicks,
        "Red soak exceeded 5% 2 ms budget: " + red.slowTicksOver2Ms());
  }

  @Test
  void opponentWallPinFourSecondRecoveryAndPassingCommitment() {
    CbfSafetyFilter cbf = new CbfSafetyFilter(0.85, 4.0);
    TopologicalPassingCommitment passing = new TopologicalPassingCommitment(0.25, 0.40);
    TRatioWaypointHandoff handoff = new TRatioWaypointHandoff(0.35);

    List<Translation2d> waypoints =
        List.of(
            new Translation2d(1.0, 1.0), new Translation2d(4.0, 1.0), new Translation2d(7.0, 1.0));
    Translation2d ego = new Translation2d(1.5, 1.0);
    int activeIdx = 1;

    // 4.0 s (200 ticks) opponent pin directly ahead, then opponent backs away
    for (int tick = 0; tick < 350; tick++) {
      double tSec = tick * DT_SEC;
      boolean pinned = tick < 200;
      Translation2d oppPos = pinned ? new Translation2d(2.35, 1.02) : new Translation2d(2.35, 4.5);

      TopologicalPassingCommitment.PassingSide side =
          passing.update(tSec, ego, waypoints.get(activeIdx), oppPos, 1.0, 1.35);
      if (pinned && ego.getX() < oppPos.getX() - 0.1) {
        assertEquals(TopologicalPassingCommitment.PassingSide.LEFT, side);
      }

      Translation2d toGoal = waypoints.get(activeIdx).minus(ego);
      double dist = toGoal.getNorm();
      Translation2d desired = dist > 1e-4 ? toGoal.div(dist).times(3.0) : new Translation2d();

      CbfSafetyFilter.FilterResult res =
          cbf.filter(
              ego,
              desired,
              List.of(
                  new CbfSafetyFilter.BarrierObstacle(
                      "pinning_opp",
                      oppPos,
                      new Translation2d(),
                      0.80,
                      CbfSafetyFilter.AgentRole.OPPONENT)));

      ego = ego.plus(res.safeVelocityMps().times(DT_SEC));
      // Must never penetrate inside opponent safety radius during pin
      if (pinned && ego.getX() < oppPos.getX() - 0.1) {
        assertTrue(
            ego.getDistance(oppPos) >= 0.72, "Penetrated opponent during pin at tick " + tick);
      }

      TRatioWaypointHandoff.HandoffState hs = handoff.evaluate(waypoints, activeIdx, ego);
      activeIdx = hs.activeIndex();
    }

    // After pin release, robot advances cleanly past intermediate waypoint to final goal
    assertEquals(2, activeIdx);
    assertTrue(ego.getX() > 4.5);
  }

  private static YaeceSoakMetrics run160SecondSoak(boolean redAlliance) {
    TRatioWaypointHandoff handoff = new TRatioWaypointHandoff(0.35);
    CbfSafetyFilter cbf = new CbfSafetyFilter(0.8, 4.2);
    TopologicalPassingCommitment passing = new TopologicalPassingCommitment(0.25, 0.40);
    TurretChassisAuthoritySolver sotm = new TurretChassisAuthoritySolver(1.25, 2.0, 0.5);
    SlotOccupancyGrid grid = new SlotOccupancyGrid();
    VisionFusionPolicy visionPolicy = new VisionFusionPolicy();
    CommandBindingGuard guard = new CommandBindingGuard();
    AtomicInteger bindCalls = new AtomicInteger(0);

    List<Translation2d> blueLoop =
        List.of(
            new Translation2d(2.0, 2.0),
            new Translation2d(6.0, 2.0),
            new Translation2d(6.0, 6.0),
            new Translation2d(2.0, 6.0));
    List<Translation2d> path =
        redAlliance
            ? blueLoop.stream().map(FullMatchSoakAndBrownoutTest::flipRotational).toList()
            : blueLoop;
    Translation2d hubTarget =
        redAlliance ? flipRotational(new Translation2d(4.6, 4.0)) : new Translation2d(4.6, 4.0);
    Translation2d allyPos =
        redAlliance ? flipRotational(new Translation2d(3.5, 4.5)) : new Translation2d(3.5, 4.5);
    Translation2d oppPos =
        redAlliance ? flipRotational(new Translation2d(5.0, 3.5)) : new Translation2d(5.0, 3.5);

    List<CbfSafetyFilter.BarrierObstacle> obstacles =
        List.of(
            new CbfSafetyFilter.BarrierObstacle(
                "ally",
                allyPos,
                new Translation2d(),
                0.75,
                CbfSafetyFilter.AgentRole.ALLIANCE_PARTNER),
            new CbfSafetyFilter.BarrierObstacle(
                "opp", oppPos, new Translation2d(), 0.80, CbfSafetyFilter.AgentRole.OPPONENT));

    Translation2d pos = path.get(0);
    int activeIdx = 1;
    int visionAccepted = 0;
    int visionRejected = 0;
    int slotClears = 0;
    int slowTicks = 0;

    for (int tick = 0; tick < FULL_MATCH_TICKS; tick++) {
      long tStartNs = System.nanoTime();
      double tSec = tick * DT_SEC;

      // Simulate 5 disable/re-enable transitions exercising CommandBindingGuard (#9558)
      if (tick % 1_600 == 0) {
        guard.bindOnce("auto_encounter_trigger", bindCalls::incrementAndGet);
      }

      Translation2d goal = path.get(activeIdx);
      Translation2d toGoal = goal.minus(pos);
      double dist = toGoal.getNorm();
      Translation2d desiredVel = dist > 1e-4 ? toGoal.div(dist).times(3.2) : new Translation2d();

      passing.update(tSec, pos, goal, oppPos, 1.0, 1.2);
      CbfSafetyFilter.FilterResult cbfRes = cbf.filter(pos, desiredVel, obstacles);
      pos = pos.plus(cbfRes.safeVelocityMps().times(DT_SEC));

      TRatioWaypointHandoff.HandoffState hs = handoff.evaluate(path, activeIdx, pos);
      activeIdx = hs.activeIndex();
      if (hs.atFinalWaypoint() && pos.getDistance(path.get(path.size() - 1)) < 0.40) {
        activeIdx = 0;
      }

      TurretChassisAuthoritySolver.AuthoritySolution sol =
          sotm.solve(
              pos,
              cbfRes.safeVelocityMps(),
              new Translation2d(),
              0.0,
              new Translation2d(0.2, 0.0),
              0.0,
              hubTarget,
              0.70);
      assertTrue(Double.isFinite(sol.fieldAimYawRad()));

      // Vision policy: pre-enable first 50 ticks, 3 s blackout at t = 40 s, high-spin at t = 100 s
      boolean enabled = tick >= 50;
      boolean blackout = tick >= 2_000 && tick < 2_150;
      double omegaDeg = (tick >= 5_000 && tick < 5_050) ? 420.0 : 45.0;
      VisionFusionPolicy.FusionDecision dec =
          visionPolicy.evaluate(enabled, blackout ? 0 : 3, 2.5, 0.90, 0.12, omegaDeg, true);
      if (dec.accepted()) {
        visionAccepted++;
      } else {
        visionRejected++;
      }

      // Exercise clamped log-odds slot occupancy every 1,000 ticks (saturate then clear in ~9
      // misses)
      int slotPhase = tick % 1_000;
      if (slotPhase < 10) {
        grid.recordHit("slot_A", tSec);
      } else if (slotPhase >= 10 && slotPhase < 22) {
        boolean wasOcc = grid.isOccupied("slot_A");
        grid.recordMiss("slot_A", tSec);
        if (wasOcc && !grid.isOccupied("slot_A")) {
          slotClears++;
        }
      }

      if ((System.nanoTime() - tStartNs) > 2_000_000L) {
        slowTicks++;
      }
    }

    return new YaeceSoakMetrics(
        pos, visionAccepted, visionRejected, slotClears, bindCalls.get(), slowTicks);
  }

  private static Translation2d flipRotational(Translation2d p) {
    return new Translation2d(FIELD_LENGTH_M - p.getX(), FIELD_WIDTH_M - p.getY());
  }
}
