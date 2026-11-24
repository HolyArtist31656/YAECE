package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.console.PlayMode;
import yaece.ledger.OptimizationObjective;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.replay.RecordedTick;
import yaece.replay.YaeceInputRecorder;
import yaece.replay.YaeceReplayHarness;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;

class Phase7SotmAlignAndReplayTest {

  @Test
  void polarAutoAlignDeratesByExtensionHonorsOmegaOverrideAndHoldsConvergence() {
    PolarAutoAlignController controller = new PolarAutoAlignController();
    Pose2d start = new Pose2d(1.0, 2.0, new Rotation2d());
    Pose2d goal = new Pose2d(4.0, 5.0, Rotation2d.fromRadians(0.5));

    // Straight-line direction (dx == dy -> vx == vy) and omegaOverride honored
    var lowCg = controller.calculate(start, goal, 1.00, 0.0, OptionalDouble.of(-1.25));
    assertFalse(lowCg.atGoal());
    assertEquals(lowCg.vxFieldMps(), lowCg.vyFieldMps(), 1e-4);
    assertEquals(-1.25, lowCg.omegaRadPerSec(), 1e-6);

    // High mechanism extension (95%) derates max linear speed
    PolarAutoAlignController extendedCtrl = new PolarAutoAlignController();
    for (int i = 0; i < 40; i++) {
      extendedCtrl.calculate(start, goal, i * 0.02, 0.95, OptionalDouble.empty());
    }
    var derated = extendedCtrl.calculate(start, goal, 0.82, 0.95, OptionalDouble.empty());
    double deratedSpeed = Math.hypot(derated.vxFieldMps(), derated.vyFieldMps());
    assertTrue(
        deratedSpeed <= 1.8, "Extended mechanism should derate linear speed, got " + deratedSpeed);

    // Convergence hold timer: single tick on goal does not immediately assert atGoal
    PolarAutoAlignController settleCtrl = new PolarAutoAlignController();
    var t0 = settleCtrl.calculate(goal, goal, 1.00, 0.0, OptionalDouble.empty());
    assertFalse(t0.atGoal(), "Must hold within tolerance for convergenceHoldSec");
    PolarAutoAlignController.ChassisVelocityCommand settled = t0;
    for (int i = 1; i <= 8; i++) {
      settled = settleCtrl.calculate(goal, goal, 1.00 + i * 0.02, 0.0, OptionalDouble.empty());
    }
    assertTrue(settled.atGoal(), "Should assert atGoal after holding >= 0.12 s");

    // Safety timeout (> 5.0 s)
    var timedOut = settleCtrl.calculate(start, goal, 7.00, 0.0, OptionalDouble.empty());
    assertTrue(timedOut.timedOut());
  }

  @Test
  void shootOnTheMoveSolverLeadsMovingTargetWithDragAndExclusionZone() {
    List<Translation2d> hubBoxOfBad =
        List.of(
            new Translation2d(4.0, 3.4),
            new Translation2d(5.2, 3.4),
            new Translation2d(5.2, 4.6),
            new Translation2d(4.0, 4.6));

    // Empirical TOF lookup: ~0.22 s/m
    ShootOnTheMoveSolver dragSolver =
        new ShootOnTheMoveSolver(
            ShootOnTheMoveSolver.Config.REBUILT_DEFAULT,
            d -> 0.15 + 0.18 * d,
            List.of(hubBoxOfBad));
    ShootOnTheMoveSolver vacuumSolver =
        new ShootOnTheMoveSolver(
            new ShootOnTheMoveSolver.Config(0.03, 0.0, 20, new Translation2d(), 1.2, 8.5),
            d -> 0.15 + 0.18 * d,
            List.of());

    Pose2d ego = new Pose2d(2.0, 4.0, new Rotation2d());
    Translation3d hubTarget = new Translation3d(4.59, 4.035, 1.83);

    // Robot strafing +Y at 2.5 m/s -> virtual target must lead in -Y
    var solDrag = dragSolver.solve(ego, 0.0, 2.5, 0.0, hubTarget);
    var solVac = vacuumSolver.solve(ego, 0.0, 2.5, 0.0, hubTarget);

    assertTrue(solDrag.virtualTargetField().getY() < hubTarget.getY());
    assertTrue(
        solDrag.effectiveTofSec() < solVac.effectiveTofSec(),
        "Exponential drag effectiveTOF must be strictly less than vacuum TOF");
    assertTrue(solDrag.inRange());
    assertFalse(solDrag.insideExclusionZone());
    assertTrue(
        solDrag.aimOmegaFfRadPerSec() < 0.0, "Strafing +Y requires negative aiming omega FF");

    // Inside exclusion polygon ("box of bad")
    var insideBad =
        dragSolver.solve(new Pose2d(4.5, 4.0, new Rotation2d()), 0.0, 0.0, 0.0, hubTarget);
    assertTrue(insideBad.insideExclusionZone());
  }

  @Test
  void inputRecorderAndReplayHarnessProduceBitIdenticalRecommendations() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    YaeceEngine liveEngine =
        YaeceEngine.builder(rebuilt)
            .withAlliance(AllianceSide.BLUE)
            .withKinematics(RobotKinematicsSpec.FRC_SWERVE_DEFAULT)
            .build();
    YaeceInputRecorder recorder = new YaeceInputRecorder();
    List<TacticalRecommendation> liveOutputs = new ArrayList<>();

    for (int i = 0; i < 15; i++) {
      double t = 1.0 + i * 0.02;
      double rem = 120.0 - i * 0.02;
      boolean hubActive = i < 8;
      Pose3d ego = new Pose3d(2.5 + i * 0.03, 4.0, 0.0, new Rotation3d());
      List<GamePieceObservation6DoF> pieces =
          List.of(
              GamePieceObservation6DoF.field6DoF(
                  0, new Pose3d(6.5, 3.2, 0.075, new Rotation3d()), 0.94, t));
      List<RobotObservation> robots =
          List.of(
              RobotObservation.fieldRobot(
                  new Pose3d(11.0, 4.5, 0.0, new Rotation3d()),
                  RobotAffiliation.OPPONENT,
                  0.92,
                  t));
      Map<String, Integer> held = Map.of("fuel", 2);
      Map<String, Boolean> dyn = Map.of("hub_active", hubActive);

      liveEngine.setPlayMode(PlayMode.OFFENSE);
      liveEngine.setDefenseStrategyMode(DefenseStrategyMode.AUTO_ADAPTIVE);
      liveEngine.setOptimizationObjective(OptimizationObjective.RANKING_POINTS);
      liveEngine.setDynamicKeyActive("hub_active", hubActive);
      liveEngine.updateEgoPose(ego, t);
      liveEngine.feedGamePieces(pieces);
      liveEngine.feedRobots(robots);

      TacticalRecommendation rec =
          liveEngine.updateAndEvaluate(
              t, new RobotInventoryState(held, true, true), MatchPeriod.TELEOP, rem);
      liveOutputs.add(rec);

      recorder.record(
          new RecordedTick(
              t,
              rem,
              MatchPeriod.TELEOP,
              ego,
              pieces,
              robots,
              held,
              PlayMode.OFFENSE,
              DefenseStrategyMode.AUTO_ADAPTIVE,
              OptimizationObjective.RANKING_POINTS,
              dyn));
    }

    String jsonl = recorder.toJsonLines();
    List<RecordedTick> parsed = YaeceInputRecorder.parseJsonLines(jsonl);
    assertEquals(15, parsed.size());

    List<TacticalRecommendation> replayed =
        YaeceReplayHarness.replay(
            rebuilt, AllianceSide.BLUE, RobotKinematicsSpec.FRC_SWERVE_DEFAULT, parsed);
    assertEquals(liveOutputs.size(), replayed.size());

    for (int i = 0; i < liveOutputs.size(); i++) {
      assertEquals(
          liveOutputs.get(i).intent(), replayed.get(i).intent(), "Intent mismatch at tick " + i);
      assertEquals(
          liveOutputs.get(i).targetId(),
          replayed.get(i).targetId(),
          "Target mismatch at tick " + i);
      assertEquals(
          liveOutputs.get(i).utilityScore(),
          replayed.get(i).utilityScore(),
          1e-3,
          "Utility mismatch at tick " + i);
    }
  }
}
