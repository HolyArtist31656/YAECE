// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;
import yaece.ballistics.TurretChassisAuthoritySolver;
import yaece.navigation.TRatioWaypointHandoff;

/**
 * Waypoint B (v2027.6.5-alpha1) tests covering BLine t-ratio waypoint handoff, closed-form CBF-QP
 * doorway traversal, Nash vs. Stackelberg avoidance weighting, COMPASS topological passing
 * commitment, SOTM -phi sign and 1/d chassis authority sharing, and seeded SplittableRandom fuzz
 * invariants.
 */
class WaypointBControlAndFuzzTest {

  @Test
  void tRatioHandoffAdvancesLaterallyBumpedRobotWithoutReversing() {
    TRatioWaypointHandoff handoff = new TRatioWaypointHandoff(0.35);
    List<Translation2d> path =
        List.of(
            new Translation2d(0.0, 0.0), new Translation2d(2.0, 0.0), new Translation2d(4.0, 0.0));

    // Robot bumped 0.8 m laterally off-line at x = 1.75 m (past 1 - 0.35/2.0 = 0.825 -> 1.65 m)
    TRatioWaypointHandoff.HandoffState state =
        handoff.evaluate(path, 1, new Translation2d(1.75, 0.80));
    assertTrue(state.advanced());
    assertEquals(2, state.activeIndex());
    assertTrue(state.atFinalWaypoint());

    // Before threshold (x = 1.0 m, y = 0.8 m): stays on waypoint 1
    TRatioWaypointHandoff.HandoffState before =
        handoff.evaluate(path, 1, new Translation2d(1.0, 0.80));
    assertFalse(before.advanced());
    assertEquals(1, before.activeIndex());
  }

  @Test
  void cbfSafetyFilterTraversesNarrowDoorwayWithoutOscillation() {
    CbfSafetyFilter cbf = new CbfSafetyFilter(0.8, 4.0);
    // Two posts forming a 0.70 m clear doorway centered on y = 0 at x = 2.0 m
    // Combined radius = 0.35 m each at y = +/- 0.70 m -> clear center corridor
    List<CbfSafetyFilter.BarrierObstacle> doorway =
        List.of(
            new CbfSafetyFilter.BarrierObstacle(
                "left_post",
                new Translation2d(2.0, 0.72),
                new Translation2d(),
                0.36,
                CbfSafetyFilter.AgentRole.STATIC_OBSTACLE),
            new CbfSafetyFilter.BarrierObstacle(
                "right_post",
                new Translation2d(2.0, -0.72),
                new Translation2d(),
                0.36,
                CbfSafetyFilter.AgentRole.STATIC_OBSTACLE));

    Translation2d pos = new Translation2d(0.0, 0.05);
    double dt = 0.02;
    for (int step = 0; step < 150; step++) {
      Translation2d desired = new Translation2d(2.0, 0.0);
      CbfSafetyFilter.FilterResult res = cbf.filter(pos, desired, doorway);
      // Forward progress remains positive without doorway stall/oscillation
      assertTrue(res.safeVelocityMps().getX() > 0.5, "Stalled at step " + step);
      pos = pos.plus(res.safeVelocityMps().times(dt));
    }
    assertTrue(pos.getX() > 3.5, "Robot should traverse cleanly through doorway");
  }

  @Test
  void cbfDifferentiatesNashAllianceVsStackelbergOpponentResponsibility() {
    CbfSafetyFilter cbf = new CbfSafetyFilter(0.8, 4.0);
    Translation2d ego = new Translation2d(0.0, 0.0);
    Translation2d desired = new Translation2d(3.0, 0.0);

    CbfSafetyFilter.BarrierObstacle ally =
        new CbfSafetyFilter.BarrierObstacle(
            "ally1",
            new Translation2d(1.2, 0.0),
            new Translation2d(),
            1.0,
            CbfSafetyFilter.AgentRole.ALLIANCE_PARTNER);
    CbfSafetyFilter.BarrierObstacle opp =
        new CbfSafetyFilter.BarrierObstacle(
            "opp1",
            new Translation2d(1.2, 0.0),
            new Translation2d(),
            1.0,
            CbfSafetyFilter.AgentRole.OPPONENT);

    CbfSafetyFilter.FilterResult allyRes = cbf.filter(ego, desired, List.of(ally));
    CbfSafetyFilter.FilterResult oppRes = cbf.filter(ego, desired, List.of(opp));

    assertTrue(allyRes.filterActive());
    assertTrue(oppRes.filterActive());
    // Stackelberg (100% ego) brakes harder than Nash (50/50 ally split)
    assertTrue(oppRes.safeVelocityMps().getX() < allyRes.safeVelocityMps().getX());
  }

  @Test
  void compassTopologicalPassingCommitmentEnforcesDwellAndHysteresis() {
    TopologicalPassingCommitment commitment = new TopologicalPassingCommitment(0.25, 0.40);
    Translation2d ego = new Translation2d(0.0, 0.0);
    Translation2d goal = new Translation2d(5.0, 0.0);
    Translation2d obs = new Translation2d(2.0, 0.0);

    // Initial commitment chooses LEFT (lower cost 1.0 vs 1.2)
    assertEquals(
        TopologicalPassingCommitment.PassingSide.LEFT,
        commitment.update(0.0, ego, goal, obs, 1.0, 1.2));

    // Within 0.40 s dwell time: refuses to flip even if RIGHT becomes much cheaper
    assertEquals(
        TopologicalPassingCommitment.PassingSide.LEFT,
        commitment.update(0.20, ego, goal, obs, 2.0, 0.5));

    // After dwell time, slight cost difference (< 25% hysteresis) still holds LEFT
    assertEquals(
        TopologicalPassingCommitment.PassingSide.LEFT,
        commitment.update(0.50, ego, goal, obs, 1.15, 1.0));

    // After dwell time, > 25% cheaper RIGHT flips cleanly
    assertEquals(
        TopologicalPassingCommitment.PassingSide.RIGHT,
        commitment.update(0.60, ego, goal, obs, 1.40, 1.0));

    // Once obstacle is behind ego, commitment clears to NONE
    assertEquals(
        TopologicalPassingCommitment.PassingSide.NONE,
        commitment.update(0.80, new Translation2d(3.0, 0.0), goal, obs, 1.0, 1.0));
  }

  @Test
  void sotmAuthoritySolverVerifiesNegativePhiSignAndChassisSharing() {
    TurretChassisAuthoritySolver solver = new TurretChassisAuthoritySolver(1.25, 2.0, 0.5);

    // Target at 45 deg (+3, +3), robot moving purely +Y (perpendicular component > 0 to left of
    // target line)
    Translation2d target = new Translation2d(3.0, 3.0);
    TurretChassisAuthoritySolver.AuthoritySolution sol =
        solver.solve(
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 2.0),
            new Translation2d(0.0, 0.0),
            0.0,
            new Translation2d(0.2, 0.0),
            0.0,
            target,
            0.8);

    // Moving +Y when target is at +45 deg gives positive radial (+sqrt(2)) and positive left
    // tangential (+sqrt(2))
    assertTrue(sol.radialVelocityMps() > 1.0);
    assertTrue(sol.tangentialVelocityMps() > 1.0);
    // Positive left tangential velocity requires negative (clockwise) omega FF
    assertTrue(sol.totalRequiredOmegaRadPerSec() < 0.0);
    assertEquals(
        sol.totalRequiredOmegaRadPerSec(),
        sol.chassisOmegaFeedforwardRadPerSec() + sol.turretOmegaFeedforwardRadPerSec(),
        1e-9);
    assertTrue(Math.abs(sol.chassisOmegaFeedforwardRadPerSec()) <= 2.0 + 1e-9);
    // Virtual target lags behind +Y motion -> virtual Y < target Y
    assertTrue(sol.virtualTargetMeters().getY() < target.getY());
  }

  @Test
  void seededPropertyFuzzCbfAlwaysRespectsMaxSpeedAndFiniteOutputs() {
    long seed = 2027060509L;
    SplittableRandom rng = new SplittableRandom(seed);
    double maxSpeed = 4.5;
    CbfSafetyFilter cbf = new CbfSafetyFilter(0.75, maxSpeed);

    for (int trial = 0; trial < 500; trial++) {
      Translation2d ego = new Translation2d(rng.nextDouble(0.0, 16.0), rng.nextDouble(0.0, 8.0));
      Translation2d desired =
          new Translation2d(rng.nextDouble(-6.0, 6.0), rng.nextDouble(-6.0, 6.0));
      List<CbfSafetyFilter.BarrierObstacle> obs =
          List.of(
              new CbfSafetyFilter.BarrierObstacle(
                  "o1",
                  new Translation2d(rng.nextDouble(0.0, 16.0), rng.nextDouble(0.0, 8.0)),
                  new Translation2d(rng.nextDouble(-3.0, 3.0), rng.nextDouble(-3.0, 3.0)),
                  rng.nextDouble(0.5, 1.5),
                  CbfSafetyFilter.AgentRole.OPPONENT),
              new CbfSafetyFilter.BarrierObstacle(
                  "a1",
                  new Translation2d(rng.nextDouble(0.0, 16.0), rng.nextDouble(0.0, 8.0)),
                  new Translation2d(rng.nextDouble(-3.0, 3.0), rng.nextDouble(-3.0, 3.0)),
                  rng.nextDouble(0.5, 1.5),
                  CbfSafetyFilter.AgentRole.ALLIANCE_PARTNER));

      CbfSafetyFilter.FilterResult res = cbf.filter(ego, desired, obs);
      double vx = res.safeVelocityMps().getX();
      double vy = res.safeVelocityMps().getY();
      assertTrue(
          Double.isFinite(vx) && Double.isFinite(vy),
          "Non-finite output seed " + seed + " trial " + trial);
      assertTrue(
          Math.hypot(vx, vy) <= maxSpeed + 1e-9,
          "Exceeded maxSpeed on seed " + seed + " trial " + trial);
    }
  }
}
