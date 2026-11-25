// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.season.SeasonConfig;

class FieldWorldModelTest {

  @Test
  void tracks6DoFGamePiecesAnd5MovingRobotsUnder2Point5MsPerTick() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    FieldWorldModel world = new FieldWorldModel(season, 0.45);

    // Warm up JIT & verify 6-DoF orientation convergence + 5-robot IMM velocity tracking
    long totalNanos = 0;
    int ticks = 100;
    for (int tick = 0; tick < ticks; tick++) {
      double t = tick * 0.02;
      world.updateEgoPose(
          new Pose3d(new Translation3d(2.0 + 0.5 * t, 4.0, 0.0), new Rotation3d()), t);

      List<GamePieceObservation6DoF> pieces = new ArrayList<>(20);
      for (int p = 0; p < 20; p++) {
        double px = 4.0 + (p % 5) * 1.4;
        double py = 1.5 + (p / 5) * 1.4;
        Rotation3d rot = new Rotation3d(0.0, 0.0, Math.toRadians(45.0));
        pieces.add(
            GamePieceObservation6DoF.field6DoF(
                0, new Pose3d(new Translation3d(px, py, 0.06), rot), 0.9, t));
      }

      List<RobotObservation> bots = new ArrayList<>(5);
      for (int b = 0; b < 5; b++) {
        RobotAffiliation aff = b < 3 ? RobotAffiliation.OPPONENT : RobotAffiliation.ALLY;
        double bx = 8.0 + b * 1.2 + 1.5 * t; // moving at +1.5 m/s in X
        double by = 1.5 + b * 1.3;
        bots.add(
            RobotObservation.fieldRobot(
                new Pose3d(new Translation3d(bx, by, 0.0), new Rotation3d()), aff, 0.92, t));
      }

      long start = System.nanoTime();
      world.addGamePieceObservations(pieces);
      world.addRobotObservations(bots);
      world.step(t);
      if (tick >= 20) {
        totalNanos += (System.nanoTime() - start);
      }
    }

    WorldSnapshot snap = world.getSnapshot();
    assertEquals(20, snap.gamePieces().size());
    assertEquals(5, snap.robots().size());
    assertEquals(3, snap.opponents().size());
    assertEquals(2, snap.allies().size());

    // Check 6-DoF yaw convergence on first piece
    assertEquals(
        Math.toRadians(45.0), snap.gamePieces().get(0).pose3d().getRotation().getZ(), 1e-2);

    // Check IMM velocity convergence on moving opponents (~1.5 m/s)
    assertEquals(1.5, snap.opponents().get(0).vxMps(), 0.25);

    double avgMs = (totalNanos / (double) (ticks - 20)) / 1e6;
    assertTrue(avgMs < 2.5, "Expected < 2.5 ms/tick for 20 pieces + 5 robots, got " + avgMs);
  }
}
