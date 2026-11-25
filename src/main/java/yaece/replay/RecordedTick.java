// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.replay;

import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose3d;
import yaece.console.PlayMode;
import yaece.ledger.OptimizationObjective;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotObservation;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;

/**
 * Immutable per-tick snapshot of all inputs fed into {@link yaece.tactics.YaeceEngine} for
 * zero-dependency deterministic offline replay (inspired by AdvantageKit's input-log discipline).
 */
public record RecordedTick(
    double fpgaTimestampSec,
    double matchTimeRemainingSec,
    MatchPeriod period,
    Pose3d egoPose3d,
    List<GamePieceObservation6DoF> gamePieces,
    List<RobotObservation> robots,
    Map<String, Integer> heldCountsByPieceId,
    PlayMode playMode,
    DefenseStrategyMode defenseStrategyMode,
    OptimizationObjective optimizationObjective,
    Map<String, Boolean> dynamicKeyActiveStates) {

  public RecordedTick {
    gamePieces = gamePieces == null ? List.of() : List.copyOf(gamePieces);
    robots = robots == null ? List.of() : List.copyOf(robots);
    heldCountsByPieceId = heldCountsByPieceId == null ? Map.of() : Map.copyOf(heldCountsByPieceId);
    dynamicKeyActiveStates =
        dynamicKeyActiveStates == null ? Map.of() : Map.copyOf(dynamicKeyActiveStates);
  }
}
