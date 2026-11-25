// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;
import yaece.bridges.AdvantageScopeTelemetryBridge;
import yaece.bridges.Coprocessor6DoFNtBridge;
import yaece.bridges.TelemetryVerbosity;
import yaece.console.ConsoleControlState;
import yaece.console.PlayMode;
import yaece.console.YaeceConsoleNtBridge;
import yaece.ledger.MatchScoreLedger;
import yaece.ledger.OptimizationObjective;
import yaece.ledger.ScoreProjection;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotObservation;
import yaece.scouting.ScoutingPriorRegistry;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;
import yaece.world.FieldWorldModel;
import yaece.world.RobotHealthState;
import yaece.world.WorldSnapshot;

/**
 * Simplified, strictly sensor-agnostic top-level facade for Yet Another Encounter Control Engine.
 *
 * <p>No specific cameras, LiDAR, or vision libraries are required—feed standardized {@link Pose3d}
 * observations from any source, toggle Offense/Defense and Match Points/Ranking Points on the fly,
 * and consume either advisory {@link TacticalRecommendation}s or driver-overridable Commands v3
 * automation.
 */
public final class YaeceEngine {
  private final SeasonConfig season;
  private final FieldWorldModel world;
  private final MatchScoreLedger ledger;
  private final EncounterArbiter arbiter;
  private final DynamicScoringState dynamicScoring = new DynamicScoringState();
  private final ScoutingPriorRegistry scoutingPriors;

  private AllianceSide ourAlliance;
  private PlayMode playMode = PlayMode.OFFENSE;
  private DefenseStrategyMode defenseStrategyMode = DefenseStrategyMode.AUTO_ADAPTIVE;
  private OptimizationObjective optimizationObjective = OptimizationObjective.MATCH_POINTS;
  private YaeceConsoleNtBridge consoleBridge = null;
  private AdvantageScopeTelemetryBridge telemetryBridge = null;
  private Coprocessor6DoFNtBridge coprocessorBridge = null;
  private volatile RobotHealthState robotHealth = RobotHealthState.NOMINAL;
  private volatile TacticalRecommendation latestRecommendation =
      TacticalRecommendation.idle(new Pose3d(), "Uninitialized");

  private YaeceEngine(
      SeasonConfig season,
      AllianceSide ourAlliance,
      RobotKinematicsSpec kinematics,
      ScoutingPriorRegistry scoutingPriors) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.ourAlliance = Objects.requireNonNull(ourAlliance, "ourAlliance must not be null");
    this.scoutingPriors = Objects.requireNonNull(scoutingPriors, "scoutingPriors must not be null");
    this.world = new FieldWorldModel(season, kinematics.bumperRadiusMeters());
    this.ledger = new MatchScoreLedger(season);
    this.arbiter = new EncounterArbiter(season, kinematics, this.scoutingPriors);
  }

  /** Creates a builder with sensible FRC swerve defaults. */
  public static Builder builder(SeasonConfig season) {
    return new Builder(season);
  }

  public static final class Builder {
    private final SeasonConfig season;
    private AllianceSide alliance = AllianceSide.BLUE;
    private RobotKinematicsSpec kinematics = RobotKinematicsSpec.FRC_SWERVE_DEFAULT;
    private ScoutingPriorRegistry scoutingPriors = new ScoutingPriorRegistry();

    private Builder(SeasonConfig season) {
      this.season = Objects.requireNonNull(season, "season must not be null");
    }

    public Builder withAlliance(AllianceSide alliance) {
      this.alliance = Objects.requireNonNull(alliance, "alliance must not be null");
      return this;
    }

    public Builder withKinematics(RobotKinematicsSpec kinematics) {
      this.kinematics = Objects.requireNonNull(kinematics, "kinematics must not be null");
      return this;
    }

    public Builder withScoutingPriors(ScoutingPriorRegistry scoutingPriors) {
      this.scoutingPriors =
          Objects.requireNonNull(scoutingPriors, "scoutingPriors must not be null");
      return this;
    }

    public YaeceEngine build() {
      return new YaeceEngine(season, alliance, kinematics, scoutingPriors);
    }
  }

  /** Enables automatic NetworkTables 4 console exposure under {@code yaece/console/*}. */
  public YaeceConsoleNtBridge enableNetworkTablesConsole() {
    if (this.consoleBridge == null) {
      this.consoleBridge = new YaeceConsoleNtBridge(season, ledger);
    }
    return this.consoleBridge;
  }

  /** Enables AdvantageScope 3D Struct telemetry publishing under {@code yaece/telemetry/*}. */
  public AdvantageScopeTelemetryBridge enableAdvantageScopeTelemetry(TelemetryVerbosity verbosity) {
    if (this.telemetryBridge == null) {
      this.telemetryBridge = new AdvantageScopeTelemetryBridge(verbosity);
    } else {
      this.telemetryBridge.setVerbosity(verbosity);
    }
    return this.telemetryBridge;
  }

  /**
   * Enables automatic NT4 coprocessor 6-DoF piece &amp; 2D LiDAR ingestion under {@code
   * yaece/perception/*}.
   */
  public Coprocessor6DoFNtBridge enableCoprocessorNtBridge() {
    if (this.coprocessorBridge == null) {
      this.coprocessorBridge = new Coprocessor6DoFNtBridge();
    }
    return this.coprocessorBridge;
  }

  public void setAlliance(AllianceSide alliance) {
    this.ourAlliance = Objects.requireNonNull(alliance, "alliance must not be null");
  }

  public void setPlayMode(PlayMode mode) {
    this.playMode = Objects.requireNonNull(mode, "mode must not be null");
    if (consoleBridge != null) {
      consoleBridge.setPlayMode(mode);
    }
  }

  public void togglePlayMode() {
    setPlayMode(playMode == PlayMode.OFFENSE ? PlayMode.DEFENSE : PlayMode.OFFENSE);
  }

  public void setDefenseStrategyMode(DefenseStrategyMode mode) {
    this.defenseStrategyMode = Objects.requireNonNull(mode, "mode must not be null");
    if (consoleBridge != null) {
      consoleBridge.setDefenseStrategyMode(mode);
    }
  }

  public void setOptimizationObjective(OptimizationObjective objective) {
    this.optimizationObjective = Objects.requireNonNull(objective, "objective must not be null");
    if (consoleBridge != null) {
      consoleBridge.setOptimizationObjective(objective);
    }
  }

  public void toggleOptimizationObjective() {
    setOptimizationObjective(
        optimizationObjective == OptimizationObjective.MATCH_POINTS
            ? OptimizationObjective.RANKING_POINTS
            : OptimizationObjective.MATCH_POINTS);
  }

  // --- Game-agnostic dynamic availability & score multipliers ---

  public void setDynamicKeyActive(String key, boolean active) {
    dynamicScoring.setKeyActive(key, active);
  }

  public void setDynamicKeyMultiplier(String key, double multiplier) {
    dynamicScoring.setKeyMultiplier(key, multiplier);
  }

  public void setDynamicKeyBonusPoints(String key, double flatBonusPoints) {
    dynamicScoring.setKeyBonusPoints(key, flatBonusPoints);
  }

  public void setDynamicKeyTimeRemaining(String key, double secondsRemaining) {
    dynamicScoring.setKeyTimeRemainingSec(key, secondsRemaining);
  }

  public DynamicScoringState getDynamicScoringState() {
    return dynamicScoring;
  }

  // --- Sensor-agnostic Pose3d feeds ---

  public void updateEgoPose(Pose3d egoPose3d, double timestampSec) {
    world.updateEgoPose(egoPose3d, timestampSec);
  }

  public void feedGamePieces(List<GamePieceObservation6DoF> observations) {
    world.addGamePieceObservations(observations);
  }

  public void feedRobots(List<RobotObservation> observations) {
    world.addRobotObservations(observations);
  }

  // --- Score recording & evaluation ---

  public void recordSelfScored(
      String siteId, String pieceId, MatchPeriod period, double elapsedMatchSec) {
    ledger.recordSelfScored(siteId, pieceId, period, elapsedMatchSec, dynamicScoring);
  }

  /** Steps the world model, polls NT4 console if enabled, and returns the live recommendation. */
  public TacticalRecommendation updateAndEvaluate(
      double fpgaTimestampSec,
      RobotInventoryState inventory,
      MatchPeriod period,
      double matchTimeRemainingSec) {
    if (!Double.isFinite(fpgaTimestampSec) || fpgaTimestampSec < 0.0) {
      throw new IllegalArgumentException("fpgaTimestampSec must be finite and >= 0");
    }
    double clampedRemaining =
        Math.max(0.0, Math.min(season.matchDurationSeconds(), matchTimeRemainingSec));
    if (consoleBridge != null) {
      double elapsed = Math.max(0.0, season.matchDurationSeconds() - clampedRemaining);
      ConsoleControlState cs = consoleBridge.poll(elapsed);
      this.playMode = cs.playMode();
      this.defenseStrategyMode = cs.defenseStrategyMode();
      this.optimizationObjective = cs.optimizationObjective();
    }

    if (coprocessorBridge != null) {
      coprocessorBridge.pollInto(this);
    }
    WorldSnapshot snap = world.step(fpgaTimestampSec);
    TacticalRecommendation rec =
        arbiter.evaluate(
            snap,
            ledger,
            inventory,
            ourAlliance,
            period,
            clampedRemaining,
            playMode,
            defenseStrategyMode,
            optimizationObjective,
            dynamicScoring);
    this.latestRecommendation = rec;
    if (telemetryBridge != null) {
      telemetryBridge.publish(snap, rec);
    }
    return rec;
  }

  public TacticalRecommendation getLatestRecommendation() {
    return latestRecommendation;
  }

  /** Feeds optional drivetrain odometry confidence, skid/tilt, and brownout throttle state. */
  public void updateRobotHealth(RobotHealthState health) {
    this.robotHealth = Objects.requireNonNull(health, "health must not be null");
  }

  public RobotHealthState getRobotHealth() {
    return robotHealth;
  }

  public WorldSnapshot getWorldSnapshot() {
    return world.getSnapshot();
  }

  public MatchScoreLedger getLedger() {
    return ledger;
  }

  public ScoutingPriorRegistry getScoutingPriors() {
    return scoutingPriors;
  }

  /** Binds a live tracked opponent ID to a pre-match scouted FRC team number. */
  public boolean bindOpponentTrackToTeam(int trackId, int teamNumber) {
    return scoutingPriors.bindTrackToTeam(trackId, teamNumber);
  }

  public ScoreProjection getScoreProjection(double elapsedMatchSec) {
    return ledger.projectScores(elapsedMatchSec);
  }

  public void resetForMatch() {
    world.reset();
    ledger.reset();
    arbiter.reset();
    dynamicScoring.reset();
    scoutingPriors.clearTrackBindings();
    robotHealth = RobotHealthState.NOMINAL;
    latestRecommendation = TacticalRecommendation.idle(new Pose3d(), "Match reset");
  }
}
