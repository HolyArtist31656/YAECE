package yaece.perception;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class LidarCameraRobotFuserTest {

  @Test
  void fusesLidarBumperClusterWithCameraBearingAndDegradesGracefullyWithoutCamera() {
    LidarCameraRobotFuser fuser = new LidarCameraRobotFuser();

    List<LidarCluster> clusters =
        List.of(
            new LidarCluster(2.0, 0.0, 0.85, 0.9, 10.0), // matched with opponent cam
            new LidarCluster(0.0, 3.0, 0.80, 0.85, 10.0), // no camera -> UNKNOWN obstacle
            new LidarCluster(1.5, 1.0, 0.12, 0.9, 10.0) // too narrow (pole/piece) -> filtered out
            );
    List<CameraBumperBearing> bearings =
        List.of(
            new CameraBumperBearing(
                Math.toRadians(4.0), RobotAffiliation.OPPONENT, 0.95, 2.2, 10.0));

    List<RobotObservation> fused = fuser.fuse(clusters, bearings);
    assertEquals(2, fused.size());
    assertEquals(RobotAffiliation.OPPONENT, fused.get(0).affiliation());
    assertEquals(2.0, fused.get(0).pose3d().getX(), 1e-6);
    assertEquals(RobotAffiliation.UNKNOWN, fused.get(1).affiliation());
    assertEquals(3.0, fused.get(1).pose3d().getY(), 1e-6);
  }
}
