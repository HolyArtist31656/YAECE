package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.console.PlayMode;
import yaece.ledger.MatchScoreLedger;
import yaece.ledger.OptimizationObjective;
import yaece.perception.RobotAffiliation;
import yaece.season.AllianceSide;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;
import yaece.world.TrackedGamePiece;
import yaece.world.TrackedRobot;
import yaece.world.WorldSnapshot;

class EncounterArbiterTest {

  @Test
  void prioritizesRankingPointUntilLockedThenSpillsOverToMatchPoints() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    EncounterArbiter arbiter = new EncounterArbiter(season, RobotKinematicsSpec.FRC_SWERVE_DEFAULT);
    MatchScoreLedger ledger = new MatchScoreLedger(season);

    // Robot holds BOTH 1 cylinder_core (5 teleop pts) and 1 fuel_sphere (2 teleop pts, counts
    // toward energized_rp)
    RobotInventoryState inv =
        new RobotInventoryState(Map.of("cylinder_core", 1, "fuel_sphere", 1), true, true);
    WorldSnapshot world =
        new WorldSnapshot(
            40.0,
            new Pose3d(new Translation3d(4.0, 4.0, 0.0), new Rotation3d()),
            0.0,
            0.0,
            List.of(),
            List.of(),
            Map.of());

    // 1. In MATCH_POINTS mode -> picks high_post (5 pts > 2 pts)
    TacticalRecommendation mpRec =
        arbiter.evaluate(
            world,
            ledger,
            inv,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            100.0,
            PlayMode.OFFENSE,
            DefenseStrategyMode.AUTO_ADAPTIVE,
            OptimizationObjective.MATCH_POINTS);
    assertEquals(EncounterIntent.SCORE, mpRec.intent());
    assertEquals("cylinder_core", mpRec.pieceId());

    // 2. In RANKING_POINTS mode with 15/18 fuel scored -> switches to alliance_hub to finish
    // energized_rp!
    ledger.updateObservedAllianceObjectiveCount("energized_rp", 15, 39.0);
    arbiter.reset();
    TacticalRecommendation rpRec =
        arbiter.evaluate(
            world,
            ledger,
            inv,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            100.0,
            PlayMode.OFFENSE,
            DefenseStrategyMode.AUTO_ADAPTIVE,
            OptimizationObjective.RANKING_POINTS);
    assertEquals(EncounterIntent.SCORE, rpRec.intent());
    assertEquals("alliance_hub", rpRec.targetId());

    // 3. Once 18/18 fuel reached -> RP is locked, automatically spills back over to cylinder_core!
    ledger.updateObservedAllianceObjectiveCount("energized_rp", 18, 41.0);
    arbiter.reset();
    TacticalRecommendation spillRec =
        arbiter.evaluate(
            world,
            ledger,
            inv,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            95.0,
            PlayMode.OFFENSE,
            DefenseStrategyMode.AUTO_ADAPTIVE,
            OptimizationObjective.RANKING_POINTS);
    assertEquals(EncounterIntent.SCORE, spillRec.intent());
    assertEquals("cylinder_core", spillRec.pieceId());
  }

  @Test
  void abortsContestedGamePieceAndAligns6DoFYawToUncontestedPiece() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    EncounterArbiter arbiter = new EncounterArbiter(season, RobotKinematicsSpec.FRC_SWERVE_DEFAULT);
    MatchScoreLedger ledger = new MatchScoreLedger(season);

    // Piece #1 is at (6.0, 4.0) but an opponent is right next to it at (6.2, 4.0) -> lost race!
    // Piece #2 is at (7.0, 2.0) uncontested with 6-DoF yaw = 60 deg
    TrackedGamePiece contested =
        new TrackedGamePiece(
            1,
            "cylinder_core",
            0,
            new Pose3d(new Translation3d(6.0, 4.0, 0.06), new Rotation3d()),
            new Translation3d(),
            0.05,
            0.9,
            1.0,
            10.0);
    TrackedGamePiece uncontested =
        new TrackedGamePiece(
            2,
            "cylinder_core",
            0,
            new Pose3d(
                new Translation3d(4.0, 1.5, 0.06), new Rotation3d(0.0, 0.0, Math.toRadians(60.0))),
            new Translation3d(),
            0.05,
            0.95,
            1.0,
            10.0);
    TrackedRobot oppNearPiece1 =
        new TrackedRobot(
            9,
            RobotAffiliation.OPPONENT,
            new Pose3d(new Translation3d(6.2, 4.0, 0.0), new Rotation3d()),
            0.0,
            0.0,
            0.45,
            0.5,
            0.08,
            1.0,
            false,
            10.0);

    WorldSnapshot world =
        new WorldSnapshot(
            10.0,
            new Pose3d(new Translation3d(2.0, 4.0, 0.0), new Rotation3d()),
            0.0,
            0.0,
            List.of(contested, uncontested),
            List.of(oppNearPiece1),
            Map.of());

    TacticalRecommendation rec =
        arbiter.evaluate(
            world,
            ledger,
            RobotInventoryState.empty(),
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            110.0,
            PlayMode.OFFENSE,
            DefenseStrategyMode.AUTO_ADAPTIVE,
            OptimizationObjective.MATCH_POINTS);

    assertEquals(EncounterIntent.GATHER, rec.intent());
    assertEquals("piece:2", rec.targetId());
    assertEquals(Math.toRadians(60.0), rec.targetPose3d().getRotation().getZ(), 1e-6);
  }

  @Test
  void fallsBackToHoldStageWhenScoringSiteApproachIsBlockedByOpponent() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    EncounterArbiter arbiter = new EncounterArbiter(season, RobotKinematicsSpec.FRC_SWERVE_DEFAULT);
    MatchScoreLedger ledger = new MatchScoreLedger(season);

    // Block BOTH high_post_left approach (2.60, 5.50) and high_post_right approach (2.60, 2.57)
    TrackedRobot blockerLeft =
        new TrackedRobot(
            1,
            RobotAffiliation.OPPONENT,
            new Pose3d(new Translation3d(2.60, 5.50, 0.0), new Rotation3d()),
            0.0,
            0.0,
            0.45,
            0.95,
            0.05,
            1.0,
            false,
            20.0);
    TrackedRobot blockerRight =
        new TrackedRobot(
            2,
            RobotAffiliation.OPPONENT,
            new Pose3d(new Translation3d(2.60, 2.57, 0.0), new Rotation3d()),
            0.0,
            0.0,
            0.45,
            0.95,
            0.05,
            1.0,
            false,
            20.0);

    WorldSnapshot world =
        new WorldSnapshot(
            20.0,
            new Pose3d(new Translation3d(5.0, 5.50, 0.0), new Rotation3d()),
            0.0,
            0.0,
            List.of(),
            List.of(blockerLeft, blockerRight),
            Map.of());

    TacticalRecommendation rec =
        arbiter.evaluate(
            world,
            ledger,
            RobotInventoryState.holding("cylinder_core"),
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            90.0,
            PlayMode.OFFENSE,
            DefenseStrategyMode.AUTO_ADAPTIVE,
            OptimizationObjective.MATCH_POINTS);

    assertEquals(EncounterIntent.HOLD_STAGE, rec.intent());
    assertNotEquals("", rec.targetId());
  }
}
