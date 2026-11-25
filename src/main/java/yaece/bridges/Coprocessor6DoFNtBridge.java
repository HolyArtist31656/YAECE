// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.bridges;

import java.util.List;
import java.util.Objects;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import yaece.perception.CameraBumperBearing;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.LidarCameraRobotFuser;
import yaece.perception.LidarCluster;
import yaece.perception.RobotObservation;
import yaece.tactics.YaeceEngine;

/**
 * Subscribes to standardized coprocessor NT4 topics under {@code yaece/perception/*}, fuses 2D
 * LiDAR clusters with multi-camera bumper bearings, and feeds both 6-DoF game pieces and robot
 * observations directly into {@link YaeceEngine}.
 */
public final class Coprocessor6DoFNtBridge implements AutoCloseable {
  private static final double[] EMPTY = new double[0];

  private final DoubleArraySubscriber piecesSub;
  private final DoubleArraySubscriber lidarSub;
  private final DoubleArraySubscriber bearingsSub;
  private final LidarCameraRobotFuser robotFuser;

  public Coprocessor6DoFNtBridge() {
    this(NetworkTableInstance.getDefault(), "yaece/perception", new LidarCameraRobotFuser());
  }

  public Coprocessor6DoFNtBridge(
      NetworkTableInstance nt, String tablePath, LidarCameraRobotFuser robotFuser) {
    Objects.requireNonNull(nt, "nt must not be null");
    this.robotFuser = Objects.requireNonNull(robotFuser, "robotFuser must not be null");
    NetworkTable table = nt.getTable(tablePath);
    this.piecesSub = table.getDoubleArrayTopic("pieces6dof").subscribe(EMPTY);
    this.lidarSub = table.getDoubleArrayTopic("lidarClusters").subscribe(EMPTY);
    this.bearingsSub = table.getDoubleArrayTopic("cameraBearings").subscribe(EMPTY);
  }

  /** Polls latest coprocessor arrays and feeds decoded observations into {@code engine}. */
  public void pollInto(YaeceEngine engine) {
    Objects.requireNonNull(engine, "engine must not be null");
    List<GamePieceObservation6DoF> pieces = CoprocessorWireCodec.decodePieces6DoF(piecesSub.get());
    if (!pieces.isEmpty()) {
      engine.feedGamePieces(pieces);
    }

    List<LidarCluster> clusters = CoprocessorWireCodec.decodeLidarClusters(lidarSub.get());
    List<CameraBumperBearing> bearings =
        CoprocessorWireCodec.decodeCameraBearings(bearingsSub.get());
    if (!clusters.isEmpty() || !bearings.isEmpty()) {
      List<RobotObservation> fused = robotFuser.fuse(clusters, bearings);
      if (!fused.isEmpty()) {
        engine.feedRobots(fused);
      }
    }
  }

  @Override
  public void close() {
    piecesSub.close();
    lidarSub.close();
    bearingsSub.close();
  }
}
