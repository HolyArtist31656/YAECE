// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.replay;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import yaece.season.AllianceSide;
import yaece.season.SeasonConfig;
import yaece.tactics.RobotInventoryState;
import yaece.tactics.RobotKinematicsSpec;
import yaece.tactics.TacticalRecommendation;
import yaece.tactics.YaeceEngine;

/**
 * Deterministic offline replay harness that feeds a recorded sequence of {@link RecordedTick}s
 * through a clean {@link YaeceEngine} instance for bit-identical JUnit regression verification.
 */
public final class YaeceReplayHarness {
  private YaeceReplayHarness() {}

  public static List<TacticalRecommendation> replay(
      SeasonConfig season,
      AllianceSide alliance,
      RobotKinematicsSpec kinematics,
      List<RecordedTick> ticks) {
    Objects.requireNonNull(season, "season must not be null");
    Objects.requireNonNull(alliance, "alliance must not be null");
    Objects.requireNonNull(kinematics, "kinematics must not be null");
    Objects.requireNonNull(ticks, "ticks must not be null");

    YaeceEngine engine =
        YaeceEngine.builder(season).withAlliance(alliance).withKinematics(kinematics).build();

    List<TacticalRecommendation> outputs = new ArrayList<>(ticks.size());
    for (RecordedTick tick : ticks) {
      engine.setPlayMode(tick.playMode());
      engine.setDefenseStrategyMode(tick.defenseStrategyMode());
      engine.setOptimizationObjective(tick.optimizationObjective());
      for (var entry : tick.dynamicKeyActiveStates().entrySet()) {
        engine.setDynamicKeyActive(entry.getKey(), entry.getValue());
      }
      engine.updateEgoPose(tick.egoPose3d(), tick.fpgaTimestampSec());
      engine.feedGamePieces(tick.gamePieces());
      engine.feedRobots(tick.robots());

      RobotInventoryState inv = new RobotInventoryState(tick.heldCountsByPieceId(), true, true);
      outputs.add(
          engine.updateAndEvaluate(
              tick.fpgaTimestampSec(), inv, tick.period(), tick.matchTimeRemainingSec()));
    }
    return List.copyOf(outputs);
  }
}
