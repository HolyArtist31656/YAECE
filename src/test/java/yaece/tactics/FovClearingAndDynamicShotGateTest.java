// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.season.SeasonConfig;
import yaece.world.FieldWorldModel;
import yaece.world.WorldSnapshot;

/**
 * Unit and fuzz tests for camera FOV negative-observation clearing in {@link FieldWorldModel} and
 * distance-scaled aim tolerance, post-bump hold, and auto-safe timeout in {@link
 * ShotReadinessGate}.
 */
class FovClearingAndDynamicShotGateTest {

  private static final long FUZZ_SEED = 2027384763L;
  private static final int FUZZ_TRIALS = 50;

  @Test
  void clearsStolenPieceInsideUnoccludedFovWhilePreservingOutOfFovAndOccludedPieces() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    FieldWorldModel world = new FieldWorldModel(season, 0.45);
    FieldWorldModel.CameraFovSpec frontCam =
        FieldWorldModel.CameraFovSpec.of("FrontIntakeCam", new Transform3d(), 70.0, 0.4, 5.0);

    // Ego at (2.0, 4.0) facing +X (yaw = 0)
    world.updateEgoPose(new Pose3d(new Translation3d(2.0, 4.0, 0.0), Rotation3d.ZERO), 1.00);
    // Piece A: directly in front at (4.0, 4.0) -> inside unoccluded FOV
    // Piece B: behind robot at (0.8, 4.0) -> outside FOV
    // Piece C: at (5.5, 5.4) -> inside FOV, occluded by opponent robot at (3.75, 4.7)
    world.addGamePieceObservations(
        List.of(
            GamePieceObservation6DoF.field6DoF(
                0, new Pose3d(new Translation3d(4.0, 4.0, 0.06), Rotation3d.ZERO), 0.95, 1.00),
            GamePieceObservation6DoF.field6DoF(
                0, new Pose3d(new Translation3d(0.8, 4.0, 0.06), Rotation3d.ZERO), 0.95, 1.00),
            GamePieceObservation6DoF.field6DoF(
                0, new Pose3d(new Translation3d(5.5, 5.4, 0.06), Rotation3d.ZERO), 0.95, 1.00)));
    world.addRobotObservations(
        List.of(
            RobotObservation.fieldRobot(
                new Pose3d(new Translation3d(3.75, 4.7, 0.0), Rotation3d.ZERO),
                RobotAffiliation.OPPONENT,
                0.95,
                1.00)));
    WorldSnapshot snap0 = world.step(1.00);
    assertEquals(3, snap0.gamePieces().size());

    // Frame 1 empty: single missed frame must NOT prematurely prune Piece A
    world.updateEgoPose(new Pose3d(new Translation3d(2.0, 4.0, 0.0), Rotation3d.ZERO), 1.02);
    world.recordEmptyCameraObservation(frontCam, 1.02);
    WorldSnapshot snap1 = world.step(1.02);
    assertEquals(3, snap1.gamePieces().size(), "Single missed frame should not prune yet");

    // Frame 2 empty (40 ms later): Piece A (unoccluded inside FOV) clears immediately,
    // while Piece B (outside FOV) and Piece C (occluded by opponent) remain tracked!
    world.updateEgoPose(new Pose3d(new Translation3d(2.0, 4.0, 0.0), Rotation3d.ZERO), 1.04);
    world.recordEmptyCameraObservation(frontCam, 1.04);
    WorldSnapshot snap2 = world.step(1.04);
    assertEquals(2, snap2.gamePieces().size());
    assertTrue(
        snap2.gamePieces().stream().noneMatch(p -> Math.abs(p.pose3d().getX() - 4.0) < 0.25),
        "Unoccluded in-FOV Piece A should have cleared after 2 empty frames");
    assertTrue(
        snap2.gamePieces().stream().anyMatch(p -> Math.abs(p.pose3d().getX() - 0.8) < 0.25),
        "Out-of-FOV Piece B should remain tracked");
    assertTrue(
        snap2.gamePieces().stream().anyMatch(p -> Math.abs(p.pose3d().getX() - 5.5) < 0.25),
        "Occluded Piece C should remain tracked");
  }

  @Test
  void distanceScaledAimToleranceClampsAndInterpolatesMonotonically() {
    ShotReadinessGate.DynamicGateConfig cfg = ShotReadinessGate.DynamicGateConfig.FRC_DEFAULT;
    assertEquals(10.0, ShotReadinessGate.distanceScaledHeadingToleranceDeg(0.5, cfg), 1e-6);
    assertEquals(10.0, ShotReadinessGate.distanceScaledHeadingToleranceDeg(1.5, cfg), 1e-6);
    assertEquals(6.0, ShotReadinessGate.distanceScaledHeadingToleranceDeg(3.75, cfg), 1e-6);
    assertEquals(2.0, ShotReadinessGate.distanceScaledHeadingToleranceDeg(6.0, cfg), 1e-6);
    assertEquals(2.0, ShotReadinessGate.distanceScaledHeadingToleranceDeg(10.0, cfg), 1e-6);
  }

  @Test
  void postBumpHoldBlocksTeleopForHalfSecondAndAutoTimeoutPreventsDeadlock() {
    ShotReadinessGate gate = new ShotReadinessGate();
    Pose3d ego =
        new Pose3d(new Translation3d(3.0, 4.0, 0.0), new Rotation3d(0.0, 0.0, Math.toRadians(5.0)));
    Pose3d target = new Pose3d(new Translation3d(3.0, 4.0, 0.0), Rotation3d.ZERO);

    // 5 deg heading error passes at close range (2.0 m -> ~9.1 deg tolerance) after settle
    ShotReadinessGate.ReadinessResult settling =
        gate.evaluateDynamic(ego, target, 2.0, 0.05, true, 10.00, 10.00, false, 0.0, false, false);
    assertFalse(settling.readyToScore());
    ShotReadinessGate.ReadinessResult readyClose =
        gate.evaluateDynamic(ego, target, 2.0, 0.05, true, 10.05, 10.05, false, 0.05, false, false);
    assertTrue(readyClose.readyToScore(), "5 deg error should fire at close 2.0 m range");

    // Same 5 deg error fails at far 6.0 m range (2.0 deg tolerance)
    gate.reset();
    ShotReadinessGate.ReadinessResult rejectedFar =
        gate.evaluateDynamic(ego, target, 6.0, 0.05, true, 10.10, 10.10, false, 0.05, false, false);
    assertFalse(rejectedFar.readyToScore(), "5 deg error should be rejected at 6.0 m range");

    // > 0.5 g bump shock holds teleop shot for 0.5 s
    gate.recordChassisAccelerationG(0.72, 11.00);
    assertTrue(gate.isInPostBumpHold(11.20));
    ShotReadinessGate.ReadinessResult heldTeleop =
        gate.evaluateDynamic(ego, target, 2.0, 0.05, true, 11.20, 11.20, false, 0.20, false, false);
    assertFalse(heldTeleop.readyToScore());
    assertTrue(heldTeleop.reason().contains("Post-bump hold"));

    // Auto max-hold timeout overrides post-bump hold so auto never deadlocks
    ShotReadinessGate.ReadinessResult autoRelease =
        gate.evaluateDynamic(ego, target, 2.0, 0.05, true, 11.25, 11.25, true, 0.50, false, false);
    assertTrue(
        autoRelease.readyToScore(), "Auto max-hold timeout must release despite post-bump hold");

    // At t = 11.55 (> 0.5 s after bump), hold expires cleanly
    assertFalse(gate.isInPostBumpHold(11.55));
  }

  @Test
  void fuzzDynamicShotGateIsBoundedAndReplayDeterministic() {
    SplittableRandom rng = new SplittableRandom(FUZZ_SEED);
    ShotReadinessGate gateA = new ShotReadinessGate();
    ShotReadinessGate gateB = new ShotReadinessGate();
    ShotReadinessGate.DynamicGateConfig cfg = ShotReadinessGate.DynamicGateConfig.FRC_DEFAULT;

    for (int i = 0; i < FUZZ_TRIALS; i++) {
      double t = 1.0 + i * 0.02;
      double dist = rng.nextDouble(0.2, 9.0);
      double tol = ShotReadinessGate.distanceScaledHeadingToleranceDeg(dist, cfg);
      assertTrue(
          tol >= cfg.minHeadingToleranceDeg() - 1e-9 && tol <= cfg.maxHeadingToleranceDeg() + 1e-9);

      double shock = rng.nextDouble(0.0, 0.9);
      gateA.recordChassisAccelerationG(shock, t);
      gateB.recordChassisAccelerationG(shock, t);

      Pose3d ego =
          new Pose3d(
              new Translation3d(3.0, 4.0, 0.0),
              new Rotation3d(0.0, 0.0, Math.toRadians(rng.nextDouble(-12.0, 12.0))));
      Pose3d target = new Pose3d(new Translation3d(3.0, 4.0, 0.0), Rotation3d.ZERO);
      boolean isAuto = rng.nextBoolean();
      double waitSec = rng.nextDouble(0.0, 0.6);

      ShotReadinessGate.ReadinessResult rA =
          gateA.evaluateDynamic(ego, target, dist, 0.05, true, t, t, isAuto, waitSec, false, false);
      ShotReadinessGate.ReadinessResult rB =
          gateB.evaluateDynamic(ego, target, dist, 0.05, true, t, t, isAuto, waitSec, false, false);
      assertEquals(rA, rB, "Dynamic shot gate must be bit-identical under replay");
    }
  }
}
