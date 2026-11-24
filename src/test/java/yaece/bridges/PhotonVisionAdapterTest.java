package yaece.bridges;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.CameraBumperBearing;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.season.SeasonConfig;

class PhotonVisionAdapterTest {

  @Test
  void raycastsFloorGamePieceAndComputesRobotFrameBumperBearing() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    // Camera mounted at z=1.06m, pitched down 26.565 deg (~atan(0.5)) -> hits z=0.06m at dx=2.0m
    Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.0, 0.0, 1.06),
            new Rotation3d(0.0, Math.atan(0.5), Math.toRadians(10.0)));
    PhotonVisionAdapter adapter = new PhotonVisionAdapter(season, robotToCam);

    Pose3d ego = new Pose3d(new Translation3d(2.0, 4.0, 0.0), new Rotation3d());
    Optional<GamePieceObservation6DoF> obs =
        adapter.toFloorGamePiece(0, -10.0, 0.0, 0.9, ego, 15.0);
    assertTrue(obs.isPresent());
    assertEquals(4.0, obs.get().pose3d().getX(), 0.05);
    assertEquals(4.0, obs.get().pose3d().getY(), 0.05);
    assertEquals(0.06, obs.get().pose3d().getZ(), 1e-6);

    CameraBumperBearing bearing =
        adapter.toRobotBumperBearing(15.0, RobotAffiliation.OPPONENT, 0.92, 3.0, 15.0);
    assertEquals(Math.toRadians(25.0), bearing.bearingRad(), 1e-6);
    assertEquals(RobotAffiliation.OPPONENT, bearing.affiliation());
  }
}
