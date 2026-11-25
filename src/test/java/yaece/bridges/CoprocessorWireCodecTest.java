// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.bridges;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.CameraBumperBearing;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.LidarCluster;
import yaece.perception.ObservationFrame;
import yaece.perception.RobotAffiliation;

class CoprocessorWireCodecTest {

  @Test
  void roundTrips6DoFPiecesLidarClustersAndCameraBearings() {
    GamePieceObservation6DoF piece =
        new GamePieceObservation6DoF(
            0,
            new Pose3d(new Translation3d(3.2, 1.5, 0.06), new Rotation3d(0.05, -0.02, 1.1)),
            ObservationFrame.ROBOT,
            0.92,
            0.88,
            true,
            12.34);
    double[] encPieces = CoprocessorWireCodec.encodePieces6DoF(List.of(piece));
    List<GamePieceObservation6DoF> decPieces = CoprocessorWireCodec.decodePieces6DoF(encPieces);
    assertEquals(1, decPieces.size());
    assertEquals(ObservationFrame.ROBOT, decPieces.get(0).frame());
    assertEquals(3.2, decPieces.get(0).pose3d().getX(), 1e-6);
    assertEquals(1.1, decPieces.get(0).pose3d().getRotation().getZ(), 1e-6);
    assertTrue(decPieces.get(0).hasFull6DoFOrientation());

    LidarCluster cluster = new LidarCluster(2.5, -0.4, 0.85, 0.9, 12.34);
    List<LidarCluster> decLidar =
        CoprocessorWireCodec.decodeLidarClusters(
            CoprocessorWireCodec.encodeLidarClusters(List.of(cluster)));
    assertEquals(1, decLidar.size());
    assertEquals(0.85, decLidar.get(0).clusterWidthMeters(), 1e-6);

    CameraBumperBearing bearing =
        new CameraBumperBearing(-0.15, RobotAffiliation.OPPONENT, 0.95, 2.6, 12.34);
    List<CameraBumperBearing> decBearings =
        CoprocessorWireCodec.decodeCameraBearings(
            CoprocessorWireCodec.encodeCameraBearings(List.of(bearing)));
    assertEquals(1, decBearings.size());
    assertEquals(RobotAffiliation.OPPONENT, decBearings.get(0).affiliation());
  }
}
