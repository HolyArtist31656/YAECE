// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.console.PlayMode;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.season.AllianceSide;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;

class YaeceEngineTest {

  @Test
  void switchesCleanlyBetweenOffenseAndDefenseOnTheFly() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    YaeceEngine engine = YaeceEngine.builder(season).withAlliance(AllianceSide.BLUE).build();

    engine.updateEgoPose(new Pose3d(new Translation3d(4.0, 4.0, 0.0), new Rotation3d()), 10.0);
    engine.feedGamePieces(
        List.of(
            GamePieceObservation6DoF.field6DoF(
                0, new Pose3d(new Translation3d(6.0, 4.0, 0.06), new Rotation3d()), 0.95, 10.0)));
    engine.feedRobots(
        List.of(
            RobotObservation.fieldRobot(
                new Pose3d(new Translation3d(11.0, 5.0, 0.0), new Rotation3d()),
                RobotAffiliation.OPPONENT,
                0.9,
                10.0)));

    // Offense mode -> gathers nearby piece
    TacticalRecommendation offRec =
        engine.updateAndEvaluate(10.0, RobotInventoryState.empty(), MatchPeriod.TELEOP, 120.0);
    assertEquals(EncounterIntent.GATHER, offRec.intent());

    // Flip to Defense on the fly -> intercepts opponent
    engine.setPlayMode(PlayMode.DEFENSE);
    TacticalRecommendation defRec =
        engine.updateAndEvaluate(10.02, RobotInventoryState.empty(), MatchPeriod.TELEOP, 119.98);
    assertEquals(EncounterIntent.DEFEND_INTERCEPT, defRec.intent());
  }
}
