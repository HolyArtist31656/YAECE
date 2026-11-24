package yaece.bridges;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import yaece.tactics.TacticalRecommendation;
import yaece.world.TrackedGamePiece;
import yaece.world.TrackedRobot;
import yaece.world.WorldSnapshot;

/**
 * Publishes live 3D {@link Pose3d} Struct arrays and tactical state under {@code yaece/telemetry/*}
 * for AdvantageScope 3D Field visualization and YACSim desktop verification.
 */
public final class AdvantageScopeTelemetryBridge implements AutoCloseable {
  private final StructPublisher<Pose3d> egoPosePub;
  private final StructPublisher<Pose3d> recommendedTargetPub;
  private final StructArrayPublisher<Pose3d> gamePiecesPub;
  private final StructArrayPublisher<Pose3d> opponentsPub;
  private final StructArrayPublisher<Pose3d> alliesPub;

  private final StringPublisher intentPub;
  private final StringPublisher targetIdPub;
  private final StringPublisher defenseStrategyPub;
  private final StringPublisher reasonPub;
  private final DoublePublisher utilityPub;
  private final BooleanPublisher pinBackoffPub;

  private TelemetryVerbosity verbosity;

  public AdvantageScopeTelemetryBridge(TelemetryVerbosity verbosity) {
    this(NetworkTableInstance.getDefault(), "yaece/telemetry", verbosity);
  }

  public AdvantageScopeTelemetryBridge(
      NetworkTableInstance nt, String tablePath, TelemetryVerbosity verbosity) {
    Objects.requireNonNull(nt, "nt must not be null");
    this.verbosity = Objects.requireNonNull(verbosity, "verbosity must not be null");
    NetworkTable table = nt.getTable(tablePath);

    this.egoPosePub = table.getStructTopic("egoPose", Pose3d.struct).publish();
    this.recommendedTargetPub = table.getStructTopic("recommendedTarget", Pose3d.struct).publish();
    this.gamePiecesPub = table.getStructArrayTopic("gamePieces", Pose3d.struct).publish();
    this.opponentsPub = table.getStructArrayTopic("opponents", Pose3d.struct).publish();
    this.alliesPub = table.getStructArrayTopic("allies", Pose3d.struct).publish();

    this.intentPub = table.getStringTopic("activeIntent").publish();
    this.targetIdPub = table.getStringTopic("activeTargetId").publish();
    this.defenseStrategyPub = table.getStringTopic("activeDefenseStrategy").publish();
    this.reasonPub = table.getStringTopic("reason").publish();
    this.utilityPub = table.getDoubleTopic("utilityScore").publish();
    this.pinBackoffPub = table.getBooleanTopic("pinBackoffRequired").publish();
  }

  public void setVerbosity(TelemetryVerbosity verbosity) {
    this.verbosity = Objects.requireNonNull(verbosity, "verbosity must not be null");
  }

  /** Publishes the current world snapshot and tactical recommendation according to verbosity. */
  public void publish(WorldSnapshot world, TacticalRecommendation rec) {
    if (verbosity == TelemetryVerbosity.OFF) {
      return;
    }

    intentPub.set(rec.intent().name());
    targetIdPub.set(rec.targetId());
    defenseStrategyPub.set(
        rec.activeDefenseStrategy() != null ? rec.activeDefenseStrategy().name() : "NONE");
    reasonPub.set(rec.reason());
    utilityPub.set(rec.utilityScore());
    pinBackoffPub.set(world.anyPinBackoffRequired());
    recommendedTargetPub.set(rec.targetPose3d());

    if (verbosity == TelemetryVerbosity.FULL_ADVANTAGESCOPE) {
      egoPosePub.set(world.egoPose3d());

      List<TrackedGamePiece> pieces = world.gamePieces();
      Pose3d[] pieceArr = new Pose3d[pieces.size()];
      for (int i = 0; i < pieces.size(); i++) {
        pieceArr[i] = pieces.get(i).pose3d();
      }
      gamePiecesPub.set(pieceArr);

      List<TrackedRobot> opps = world.opponents();
      Pose3d[] oppArr = new Pose3d[opps.size()];
      for (int i = 0; i < opps.size(); i++) {
        oppArr[i] = opps.get(i).pose3d();
      }
      opponentsPub.set(oppArr);

      List<TrackedRobot> allies = world.allies();
      Pose3d[] allyArr = new Pose3d[allies.size()];
      for (int i = 0; i < allies.size(); i++) {
        allyArr[i] = allies.get(i).pose3d();
      }
      alliesPub.set(allyArr);
    }
  }

  @Override
  public void close() {
    egoPosePub.close();
    recommendedTargetPub.close();
    gamePiecesPub.close();
    opponentsPub.close();
    alliesPub.close();
    intentPub.close();
    targetIdPub.close();
    defenseStrategyPub.close();
    reasonPub.close();
    utilityPub.close();
    pinBackoffPub.close();
  }
}
