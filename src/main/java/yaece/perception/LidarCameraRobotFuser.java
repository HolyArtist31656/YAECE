package yaece.perception;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Fuses 360-degree 2D LiDAR bumper clusters with multi-camera bumper bearing detections into
 * robot-frame {@link RobotObservation} records.
 *
 * <p>Filters LiDAR arcs by realistic FRC bumper chord width, associates each cluster with the
 * closest camera bearing within {@code maxBearingDiffRad} to stamp {@code ALLY}/{@code OPPONENT},
 * and gracefully emits {@code UNKNOWN} obstacles when cameras have blind spots or drop out.
 */
public final class LidarCameraRobotFuser {
  private final double minBumperWidthMeters;
  private final double maxBumperWidthMeters;
  private final double maxBearingDiffRad;
  private final double maxRangeMeters;

  /** Creates a fuser with standard FRC bumper dimensions (0.40–1.25 m width, 15 deg gate). */
  public LidarCameraRobotFuser() {
    this(0.40, 1.25, Math.toRadians(15.0), 8.0);
  }

  public LidarCameraRobotFuser(
      double minBumperWidthMeters,
      double maxBumperWidthMeters,
      double maxBearingDiffRad,
      double maxRangeMeters) {
    this.minBumperWidthMeters = minBumperWidthMeters;
    this.maxBumperWidthMeters = maxBumperWidthMeters;
    this.maxBearingDiffRad = maxBearingDiffRad;
    this.maxRangeMeters = maxRangeMeters;
  }

  /**
   * Fuses a batch of 2D LiDAR clusters and camera bumper bearings into robot-frame {@link
   * RobotObservation} records.
   */
  public List<RobotObservation> fuse(
      List<LidarCluster> lidarClusters, List<CameraBumperBearing> cameraBearings) {
    List<RobotObservation> out = new ArrayList<>();
    List<LidarCluster> clusters = lidarClusters == null ? List.of() : lidarClusters;
    List<CameraBumperBearing> bearings = cameraBearings == null ? List.of() : cameraBearings;
    boolean[] usedBearing = new boolean[bearings.size()];

    for (LidarCluster cluster : clusters) {
      if (cluster.clusterWidthMeters() < minBumperWidthMeters
          || cluster.clusterWidthMeters() > maxBumperWidthMeters
          || cluster.rangeMeters() > maxRangeMeters) {
        continue;
      }

      double clusterBearing = cluster.bearingRad();
      int bestIdx = -1;
      double bestDiff = maxBearingDiffRad;
      for (int i = 0; i < bearings.size(); i++) {
        if (usedBearing[i]) {
          continue;
        }
        double diff = Math.abs(wrapAngle(clusterBearing - bearings.get(i).bearingRad()));
        if (diff <= bestDiff) {
          bestDiff = diff;
          bestIdx = i;
        }
      }

      RobotAffiliation affiliation = RobotAffiliation.UNKNOWN;
      double fusedConf = cluster.confidence();
      if (bestIdx >= 0) {
        usedBearing[bestIdx] = true;
        CameraBumperBearing match = bearings.get(bestIdx);
        affiliation = match.affiliation();
        fusedConf = Math.min(1.0, 0.6 * cluster.confidence() + 0.4 * match.confidence());
      }

      double estRadius = Math.max(0.35, Math.min(0.55, 0.5 * cluster.clusterWidthMeters()));
      Pose3d robotRelPose =
          new Pose3d(
              new Translation3d(cluster.centerXMeters(), cluster.centerYMeters(), 0.0),
              new Rotation3d());
      out.add(
          new RobotObservation(
              robotRelPose,
              ObservationFrame.ROBOT,
              affiliation,
              fusedConf,
              estRadius,
              cluster.timestampSec()));
    }

    // Include unmatched camera detections that have a valid monocular range fallback
    for (int i = 0; i < bearings.size(); i++) {
      if (usedBearing[i]) {
        continue;
      }
      CameraBumperBearing b = bearings.get(i);
      if (b.estimatedRangeMeters() > 0.2 && b.estimatedRangeMeters() <= maxRangeMeters) {
        double rx = b.estimatedRangeMeters() * Math.cos(b.bearingRad());
        double ry = b.estimatedRangeMeters() * Math.sin(b.bearingRad());
        Pose3d robotRelPose = new Pose3d(new Translation3d(rx, ry, 0.0), new Rotation3d());
        out.add(
            new RobotObservation(
                robotRelPose,
                ObservationFrame.ROBOT,
                b.affiliation(),
                b.confidence() * 0.75,
                RobotObservation.DEFAULT_ROBOT_RADIUS_METERS,
                b.timestampSec()));
      }
    }

    return out;
  }

  private static double wrapAngle(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
