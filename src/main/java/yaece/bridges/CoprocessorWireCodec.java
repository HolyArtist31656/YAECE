package yaece.bridges;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.CameraBumperBearing;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.LidarCluster;
import yaece.perception.ObservationFrame;
import yaece.perception.RobotAffiliation;

/**
 * Pure-Java flat {@code double[]} wire codec for streaming 6-DoF game pieces, 2D LiDAR clusters,
 * and multi-camera bumper bearings from any Python/C++/Rust coprocessor over NetworkTables 4.
 *
 * <p>Strides:
 *
 * <ul>
 *   <li>6-DoF Piece ({@value #PIECE_STRIDE} doubles): {@code [classId, x, y, z, roll, pitch, yaw,
 *       posConf, orientConf, frameFlag(0=FIELD,1=ROBOT), timestampSec]}
 *   <li>2D LiDAR Cluster ({@value #LIDAR_STRIDE} doubles): {@code [centerX, centerY, widthMeters,
 *       confidence, timestampSec]}
 *   <li>Camera Bumper Bearing ({@value #BEARING_STRIDE} doubles): {@code [bearingRad,
 *       affiliation(0=UNKNOWN,1=ALLY,2=OPPONENT), confidence, estRangeMeters, timestampSec]}
 * </ul>
 */
public final class CoprocessorWireCodec {
  public static final int PIECE_STRIDE = 11;
  public static final int LIDAR_STRIDE = 5;
  public static final int BEARING_STRIDE = 5;

  private CoprocessorWireCodec() {}

  public static double[] encodePieces6DoF(List<GamePieceObservation6DoF> pieces) {
    if (pieces == null || pieces.isEmpty()) {
      return new double[0];
    }
    double[] out = new double[pieces.size() * PIECE_STRIDE];
    for (int i = 0; i < pieces.size(); i++) {
      GamePieceObservation6DoF p = pieces.get(i);
      int b = i * PIECE_STRIDE;
      out[b] = p.visionClassId();
      out[b + 1] = p.pose3d().getX();
      out[b + 2] = p.pose3d().getY();
      out[b + 3] = p.pose3d().getZ();
      out[b + 4] = p.pose3d().getRotation().getX();
      out[b + 5] = p.pose3d().getRotation().getY();
      out[b + 6] = p.pose3d().getRotation().getZ();
      out[b + 7] = p.positionConfidence();
      out[b + 8] = p.orientationConfidence();
      out[b + 9] = p.frame() == ObservationFrame.ROBOT ? 1.0 : 0.0;
      out[b + 10] = p.timestampSec();
    }
    return out;
  }

  public static List<GamePieceObservation6DoF> decodePieces6DoF(double[] raw) {
    if (raw == null || raw.length < PIECE_STRIDE) {
      return List.of();
    }
    int count = raw.length / PIECE_STRIDE;
    List<GamePieceObservation6DoF> out = new ArrayList<>(count);
    for (int i = 0; i < count; i++) {
      int b = i * PIECE_STRIDE;
      int classId = (int) Math.round(raw[b]);
      if (classId < 0 || !Double.isFinite(raw[b + 10]) || raw[b + 10] < 0.0) {
        continue;
      }
      Pose3d pose =
          new Pose3d(
              new Translation3d(raw[b + 1], raw[b + 2], raw[b + 3]),
              new Rotation3d(raw[b + 4], raw[b + 5], raw[b + 6]));
      ObservationFrame frame =
          Math.round(raw[b + 9]) == 1L ? ObservationFrame.ROBOT : ObservationFrame.FIELD;
      double orientConf = raw[b + 8];
      out.add(
          new GamePieceObservation6DoF(
              classId, pose, frame, raw[b + 7], orientConf, orientConf > 0.05, raw[b + 10]));
    }
    return out;
  }

  public static double[] encodeLidarClusters(List<LidarCluster> clusters) {
    if (clusters == null || clusters.isEmpty()) {
      return new double[0];
    }
    double[] out = new double[clusters.size() * LIDAR_STRIDE];
    for (int i = 0; i < clusters.size(); i++) {
      LidarCluster c = clusters.get(i);
      int b = i * LIDAR_STRIDE;
      out[b] = c.centerXMeters();
      out[b + 1] = c.centerYMeters();
      out[b + 2] = c.clusterWidthMeters();
      out[b + 3] = c.confidence();
      out[b + 4] = c.timestampSec();
    }
    return out;
  }

  public static List<LidarCluster> decodeLidarClusters(double[] raw) {
    if (raw == null || raw.length < LIDAR_STRIDE) {
      return List.of();
    }
    int count = raw.length / LIDAR_STRIDE;
    List<LidarCluster> out = new ArrayList<>(count);
    for (int i = 0; i < count; i++) {
      int b = i * LIDAR_STRIDE;
      if (raw[b + 2] <= 0.0 || raw[b + 4] < 0.0) {
        continue;
      }
      out.add(new LidarCluster(raw[b], raw[b + 1], raw[b + 2], raw[b + 3], raw[b + 4]));
    }
    return out;
  }

  public static double[] encodeCameraBearings(List<CameraBumperBearing> bearings) {
    if (bearings == null || bearings.isEmpty()) {
      return new double[0];
    }
    double[] out = new double[bearings.size() * BEARING_STRIDE];
    for (int i = 0; i < bearings.size(); i++) {
      CameraBumperBearing c = bearings.get(i);
      int b = i * BEARING_STRIDE;
      out[b] = c.bearingRad();
      out[b + 1] =
          switch (c.affiliation()) {
            case UNKNOWN -> 0.0;
            case ALLY -> 1.0;
            case OPPONENT -> 2.0;
          };
      out[b + 2] = c.confidence();
      out[b + 3] = c.estimatedRangeMeters();
      out[b + 4] = c.timestampSec();
    }
    return out;
  }

  public static List<CameraBumperBearing> decodeCameraBearings(double[] raw) {
    if (raw == null || raw.length < BEARING_STRIDE) {
      return List.of();
    }
    int count = raw.length / BEARING_STRIDE;
    List<CameraBumperBearing> out = new ArrayList<>(count);
    for (int i = 0; i < count; i++) {
      int b = i * BEARING_STRIDE;
      if (raw[b + 4] < 0.0) {
        continue;
      }
      RobotAffiliation aff =
          switch ((int) Math.round(raw[b + 1])) {
            case 1 -> RobotAffiliation.ALLY;
            case 2 -> RobotAffiliation.OPPONENT;
            default -> RobotAffiliation.UNKNOWN;
          };
      out.add(new CameraBumperBearing(raw[b], aff, raw[b + 2], raw[b + 3], raw[b + 4]));
    }
    return out;
  }
}
