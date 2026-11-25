// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.console;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.IntegerEntry;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringEntry;
import yaece.ledger.MatchScoreLedger;
import yaece.ledger.OptimizationObjective;
import yaece.ledger.ScoreProjection;
import yaece.season.DefenseStrategyMode;
import yaece.season.GamePieceSpec;
import yaece.season.ScoringObjectiveSpec;
import yaece.season.SeasonConfig;

/**
 * Zero-wiring NetworkTables 4 console bridge under {@code yaece/console/*}.
 *
 * <p>Publishes interactive dashboard entries for {@code playMode}, {@code defenseStrategy}, {@code
 * optimizationObjective}, {@code observedAllianceScore}, {@code teammateEstimatedPoints}, {@code
 * opponentObservedScore}, per-objective alliance counts, and per-piece opponent counts. Whenever a
 * human operator updates a numeric field on Elastic, AdvantageScope, or any NT4 dashboard, this
 * bridge automatically timestamps the change with {@code elapsedMatchSec} and feeds {@link
 * MatchScoreLedger}.
 */
public final class YaeceConsoleNtBridge implements AutoCloseable {
  private final MatchScoreLedger ledger;
  private final StringEntry playModeEntry;
  private final StringEntry defenseStrategyEntry;
  private final StringEntry objectiveEntry;

  private final DoubleEntry observedAllianceScoreEntry;
  private final DoubleEntry teammateEstimatedPointsEntry;
  private final DoubleEntry opponentObservedScoreEntry;

  private final Map<String, IntegerEntry> objectiveCountEntries = new HashMap<>();
  private final Map<String, IntegerEntry> opponentPieceEntries = new HashMap<>();

  private final DoublePublisher ourScorePub;
  private final DoublePublisher oppScorePub;
  private final DoublePublisher projectedMarginPub;
  private final DoublePublisher oppRatePub;

  private double lastObservedAllianceScore = -1.0;
  private double lastTeammatePoints = -1.0;
  private double lastOpponentScore = -1.0;
  private final Map<String, Long> lastObjectiveCounts = new HashMap<>();
  private final Map<String, Long> lastOpponentPieceCounts = new HashMap<>();

  private ConsoleControlState controlState = ConsoleControlState.DEFAULT;

  /**
   * Creates a console bridge on the default {@link NetworkTableInstance} under {@code
   * yaece/console}.
   */
  public YaeceConsoleNtBridge(SeasonConfig season, MatchScoreLedger ledger) {
    this(NetworkTableInstance.getDefault(), "yaece/console", season, ledger);
  }

  public YaeceConsoleNtBridge(
      NetworkTableInstance nt, String tablePath, SeasonConfig season, MatchScoreLedger ledger) {
    Objects.requireNonNull(nt, "nt must not be null");
    Objects.requireNonNull(season, "season must not be null");
    this.ledger = Objects.requireNonNull(ledger, "ledger must not be null");

    NetworkTable table = nt.getTable(tablePath);
    this.playModeEntry = table.getStringTopic("playMode").getEntry(PlayMode.OFFENSE.name());
    this.playModeEntry.set(PlayMode.OFFENSE.name());

    this.defenseStrategyEntry =
        table.getStringTopic("defenseStrategy").getEntry(DefenseStrategyMode.AUTO_ADAPTIVE.name());
    this.defenseStrategyEntry.set(DefenseStrategyMode.AUTO_ADAPTIVE.name());

    this.objectiveEntry =
        table
            .getStringTopic("optimizationObjective")
            .getEntry(OptimizationObjective.MATCH_POINTS.name());
    this.objectiveEntry.set(OptimizationObjective.MATCH_POINTS.name());

    this.observedAllianceScoreEntry = table.getDoubleTopic("observedAllianceScore").getEntry(-1.0);
    this.observedAllianceScoreEntry.set(-1.0);

    this.teammateEstimatedPointsEntry =
        table.getDoubleTopic("teammateEstimatedPoints").getEntry(-1.0);
    this.teammateEstimatedPointsEntry.set(-1.0);

    this.opponentObservedScoreEntry = table.getDoubleTopic("opponentObservedScore").getEntry(-1.0);
    this.opponentObservedScoreEntry.set(-1.0);

    NetworkTable objTable = table.getSubTable("objectiveCount");
    for (ScoringObjectiveSpec obj : season.scoringObjectives()) {
      IntegerEntry e = objTable.getIntegerTopic(obj.id()).getEntry(-1);
      e.set(-1);
      objectiveCountEntries.put(obj.id(), e);
      lastObjectiveCounts.put(obj.id(), -1L);
    }

    NetworkTable oppPieceTable = table.getSubTable("opponentPieceCount");
    for (GamePieceSpec piece : season.gamePieces()) {
      IntegerEntry e = oppPieceTable.getIntegerTopic(piece.id()).getEntry(-1);
      e.set(-1);
      opponentPieceEntries.put(piece.id(), e);
      lastOpponentPieceCounts.put(piece.id(), -1L);
    }

    NetworkTable telem = table.getSubTable("telemetry");
    this.ourScorePub = telem.getDoubleTopic("reconciledAllianceScore").publish();
    this.oppScorePub = telem.getDoubleTopic("estimatedOpponentScore").publish();
    this.projectedMarginPub = telem.getDoubleTopic("projectedFinalMargin").publish();
    this.oppRatePub = telem.getDoubleTopic("opponentScoringRatePps").publish();
  }

  /**
   * Polls all NT4 console entries, auto-timestamps any human changes into {@link MatchScoreLedger},
   * publishes live reconciled score telemetry, and returns the current {@link ConsoleControlState}.
   */
  public ConsoleControlState poll(double elapsedMatchSec) {
    PlayMode pm =
        ConsoleControlState.parseOrDefault(
            PlayMode.class, playModeEntry.get(), controlState.playMode());
    DefenseStrategyMode dm =
        ConsoleControlState.parseOrDefault(
            DefenseStrategyMode.class,
            defenseStrategyEntry.get(),
            controlState.defenseStrategyMode());
    OptimizationObjective oo =
        ConsoleControlState.parseOrDefault(
            OptimizationObjective.class,
            objectiveEntry.get(),
            controlState.optimizationObjective());
    this.controlState = new ConsoleControlState(pm, dm, oo);

    double obsAll = observedAllianceScoreEntry.get();
    if (obsAll >= 0.0 && Math.abs(obsAll - lastObservedAllianceScore) > 1e-6) {
      lastObservedAllianceScore = obsAll;
      ledger.updateObservedAllianceScore(obsAll, elapsedMatchSec);
    }

    double tmPts = teammateEstimatedPointsEntry.get();
    if (tmPts >= 0.0 && Math.abs(tmPts - lastTeammatePoints) > 1e-6) {
      lastTeammatePoints = tmPts;
      ledger.updateTeammateEstimatedPoints(tmPts, elapsedMatchSec);
    }

    double oppScore = opponentObservedScoreEntry.get();
    if (oppScore >= 0.0 && Math.abs(oppScore - lastOpponentScore) > 1e-6) {
      lastOpponentScore = oppScore;
      ledger.updateOpponentObservedScore(oppScore, elapsedMatchSec);
    }

    for (Map.Entry<String, IntegerEntry> e : objectiveCountEntries.entrySet()) {
      long val = e.getValue().get();
      if (val >= 0 && val != lastObjectiveCounts.get(e.getKey())) {
        lastObjectiveCounts.put(e.getKey(), val);
        ledger.updateObservedAllianceObjectiveCount(e.getKey(), (int) val, elapsedMatchSec);
      }
    }

    for (Map.Entry<String, IntegerEntry> e : opponentPieceEntries.entrySet()) {
      long val = e.getValue().get();
      if (val >= 0 && val != lastOpponentPieceCounts.get(e.getKey())) {
        lastOpponentPieceCounts.put(e.getKey(), val);
        ledger.updateOpponentObservedPieces(e.getKey(), (int) val, elapsedMatchSec);
      }
    }

    ScoreProjection proj = ledger.projectScores(elapsedMatchSec);
    ourScorePub.set(proj.ourCurrentScore());
    oppScorePub.set(proj.opponentCurrentScore());
    projectedMarginPub.set(proj.projectedMargin());
    oppRatePub.set(proj.opponentScoringRatePps());

    return controlState;
  }

  /** Programmatically sets PlayMode and syncs it to NetworkTables. */
  public void setPlayMode(PlayMode mode) {
    Objects.requireNonNull(mode, "mode must not be null");
    playModeEntry.set(mode.name());
    controlState =
        new ConsoleControlState(
            mode, controlState.defenseStrategyMode(), controlState.optimizationObjective());
  }

  /** Programmatically sets DefenseStrategyMode and syncs it to NetworkTables. */
  public void setDefenseStrategyMode(DefenseStrategyMode mode) {
    Objects.requireNonNull(mode, "mode must not be null");
    defenseStrategyEntry.set(mode.name());
    controlState =
        new ConsoleControlState(
            controlState.playMode(), mode, controlState.optimizationObjective());
  }

  /** Programmatically sets OptimizationObjective and syncs it to NetworkTables. */
  public void setOptimizationObjective(OptimizationObjective objective) {
    Objects.requireNonNull(objective, "objective must not be null");
    objectiveEntry.set(objective.name());
    controlState =
        new ConsoleControlState(
            controlState.playMode(), controlState.defenseStrategyMode(), objective);
  }

  public ConsoleControlState getControlState() {
    return controlState;
  }

  @Override
  public void close() {
    playModeEntry.close();
    defenseStrategyEntry.close();
    objectiveEntry.close();
    observedAllianceScoreEntry.close();
    teammateEstimatedPointsEntry.close();
    opponentObservedScoreEntry.close();
    objectiveCountEntries.values().forEach(IntegerEntry::close);
    opponentPieceEntries.values().forEach(IntegerEntry::close);
    ourScorePub.close();
    oppScorePub.close();
    projectedMarginPub.close();
    oppRatePub.close();
  }
}
