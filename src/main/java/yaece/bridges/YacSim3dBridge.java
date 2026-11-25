// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.bridges;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.console.PlayMode;
import yaece.ledger.OptimizationObjective;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.replay.RecordedTick;
import yaece.replay.YaeceInputRecorder;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.tactics.EncounterIntent;
import yaece.tactics.PolarAutoAlignController;
import yaece.tactics.RobotInventoryState;
import yaece.tactics.ShootOnTheMoveSolver;
import yaece.tactics.TacticalRecommendation;
import yaece.tactics.YaeceEngine;

/**
 * Zero-dependency 3D CAD simulation bridge connecting external physics simulators (such as YACSim)
 * to {@link YaeceEngine}, {@link PolarAutoAlignController}, {@link ShootOnTheMoveSolver}, and
 * {@link YaeceInputRecorder} without requiring any simulator JAR on the runtime classpath.
 */
public final class YacSim3dBridge {

  /** Simulated ego robot state exported from a 3D physics tick. */
  public record SimulatedRobotState(
      Pose3d pose3d,
      double vxFieldMps,
      double vyFieldMps,
      double omegaRadPerSec,
      double mechanismExtensionNormalized) {
    public SimulatedRobotState {
      Objects.requireNonNull(pose3d, "pose3d must not be null");
      mechanismExtensionNormalized = Math.max(0.0, Math.min(1.0, mechanismExtensionNormalized));
    }
  }

  /** Simulated 6-DoF field game piece pose. */
  public record SimulatedPiecePose(int classId, Pose3d pose3d) {
    public SimulatedPiecePose {
      Objects.requireNonNull(pose3d, "pose3d must not be null");
    }
  }

  /** Simulated non-ego robot pose and affiliation. */
  public record SimulatedOtherRobot(Pose3d pose3d, RobotAffiliation affiliation) {
    public SimulatedOtherRobot {
      Objects.requireNonNull(pose3d, "pose3d must not be null");
      Objects.requireNonNull(affiliation, "affiliation must not be null");
    }
  }

  /** Complete single-tick simulation frame pushed from YACSim or a desktop harness. */
  public record YacSimTickInput(
      double fpgaTimestampSec,
      double matchTimeRemainingSec,
      MatchPeriod period,
      SimulatedRobotState ego,
      List<SimulatedPiecePose> fieldPieces,
      List<SimulatedOtherRobot> otherRobots,
      RobotInventoryState inventory,
      Map<String, Boolean> dynamicKeySnapshot) {
    public YacSimTickInput {
      Objects.requireNonNull(period, "period must not be null");
      Objects.requireNonNull(ego, "ego must not be null");
      fieldPieces = fieldPieces == null ? List.of() : List.copyOf(fieldPieces);
      otherRobots = otherRobots == null ? List.of() : List.copyOf(otherRobots);
      Objects.requireNonNull(inventory, "inventory must not be null");
      dynamicKeySnapshot = dynamicKeySnapshot == null ? Map.of() : Map.copyOf(dynamicKeySnapshot);
    }
  }

  /**
   * Complete single-tick tactical, alignment, and optional SOTM output returned to the simulator.
   */
  public record YacSimTickOutput(
      TacticalRecommendation recommendation,
      Optional<PolarAutoAlignController.ChassisVelocityCommand> alignCommand,
      Optional<ShootOnTheMoveSolver.SotmSolution> sotmSolution) {}

  private final YaeceEngine engine;
  private final PolarAutoAlignController alignController;
  private final ShootOnTheMoveSolver sotmSolver;
  private final YaeceInputRecorder recorder;

  public YacSim3dBridge(YaeceEngine engine) {
    this(engine, new PolarAutoAlignController(), null, null);
  }

  public YacSim3dBridge(
      YaeceEngine engine,
      PolarAutoAlignController alignController,
      ShootOnTheMoveSolver sotmSolver,
      YaeceInputRecorder recorder) {
    this.engine = Objects.requireNonNull(engine, "engine must not be null");
    this.alignController = alignController;
    this.sotmSolver = sotmSolver;
    this.recorder = recorder;
  }

  /**
   * Steps YAECE for one simulated physics tick, optionally computes CG-derated polar alignment and
   * Shoot-on-the-Move lead solutions, and appends to {@link YaeceInputRecorder} when attached.
   */
  public YacSimTickOutput step(YacSimTickInput input) {
    Objects.requireNonNull(input, "input must not be null");
    double t = input.fpgaTimestampSec();

    List<GamePieceObservation6DoF> pieceObs = new ArrayList<>(input.fieldPieces().size());
    for (SimulatedPiecePose p : input.fieldPieces()) {
      pieceObs.add(GamePieceObservation6DoF.field6DoF(p.classId(), p.pose3d(), 1.0, t));
    }

    List<RobotObservation> robotObs = new ArrayList<>(input.otherRobots().size());
    for (SimulatedOtherRobot r : input.otherRobots()) {
      robotObs.add(RobotObservation.fieldRobot(r.pose3d(), r.affiliation(), 1.0, t));
    }

    for (Map.Entry<String, Boolean> kv : input.dynamicKeySnapshot().entrySet()) {
      engine.setDynamicKeyActive(kv.getKey(), Boolean.TRUE.equals(kv.getValue()));
    }

    engine.updateEgoPose(input.ego().pose3d(), t);
    engine.feedGamePieces(pieceObs);
    engine.feedRobots(robotObs);

    TacticalRecommendation rec =
        engine.updateAndEvaluate(
            t, input.inventory(), input.period(), input.matchTimeRemainingSec());

    Optional<ShootOnTheMoveSolver.SotmSolution> sotm = Optional.empty();
    OptionalDouble omegaOverride = OptionalDouble.empty();
    if (sotmSolver != null && rec.intent() == EncounterIntent.SCORE) {
      Translation3d target3d = rec.targetPose3d().getTranslation();
      ShootOnTheMoveSolver.SotmSolution sol =
          sotmSolver.solve(
              input.ego().pose3d().toPose2d(),
              input.ego().vxFieldMps(),
              input.ego().vyFieldMps(),
              input.ego().omegaRadPerSec(),
              target3d);
      sotm = Optional.of(sol);
      omegaOverride = OptionalDouble.of(sol.aimOmegaFfRadPerSec());
    }

    Optional<PolarAutoAlignController.ChassisVelocityCommand> alignCmd = Optional.empty();
    if (alignController != null && rec.intent() != EncounterIntent.IDLE) {
      alignCmd =
          Optional.of(
              alignController.calculate(
                  input.ego().pose3d().toPose2d(),
                  rec.targetPose3d().toPose2d(),
                  t,
                  input.ego().mechanismExtensionNormalized(),
                  omegaOverride));
    }

    if (recorder != null) {
      recorder.record(
          new RecordedTick(
              t,
              input.matchTimeRemainingSec(),
              input.period(),
              input.ego().pose3d(),
              pieceObs,
              robotObs,
              input.inventory().heldCountsByPieceId(),
              PlayMode.OFFENSE,
              DefenseStrategyMode.AUTO_ADAPTIVE,
              OptimizationObjective.MATCH_POINTS,
              input.dynamicKeySnapshot()));
    }

    return new YacSimTickOutput(rec, alignCmd, sotm);
  }
}
