package yaece.tactics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import yaece.commands3.YaeceCommandFactory;
import yaece.perception.PieceDetectionFilter;
import yaece.perception.VisionPoseGate;
import yaece.season.AllianceSide;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;
import yaece.world.EgoPoseBuffer;

class Phase6EliteUpgradesTest {

  @Test
  void fieldPathfinderCachesByEpochAndInvalidatesOnEpochBump() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    FieldPathfinder pf = new FieldPathfinder(rebuilt, 0.43);

    Translation2d start = new Translation2d(2.2, 4.03);
    Translation2d goal = new Translation2d(7.0, 4.03);

    FieldPathfinder.PathResult first =
        pf.findPath(
            start,
            goal,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            100.0,
            FieldPathfinder.ClearanceProfile.TELEOP_TIGHT);
    FieldPathfinder.PathResult cached =
        pf.findPath(
            start,
            goal,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            100.0,
            FieldPathfinder.ClearanceProfile.TELEOP_TIGHT);
    assertSame(first, cached, "Identical epoch & profile query must hit pathCache");

    FieldPathfinder.PathResult autoProfile =
        pf.findPath(
            start,
            goal,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            100.0,
            FieldPathfinder.ClearanceProfile.AUTO_TRANSIT);
    assertTrue(
        autoProfile.totalDistanceMeters() >= first.totalDistanceMeters(),
        "Wider AUTO_TRANSIT clearance profile should route at least as wide as TELEOP_TIGHT");

    pf.bumpObstacleEpoch();
    assertEquals(1L, pf.getObstacleEpoch());
    FieldPathfinder.PathResult afterBump =
        pf.findPath(
            start,
            goal,
            AllianceSide.BLUE,
            MatchPeriod.TELEOP,
            100.0,
            FieldPathfinder.ClearanceProfile.TELEOP_TIGHT);
    assertTrue(afterBump != first, "Bumping obstacleEpoch must invalidate prior cached path");
  }

  @Test
  void shotReadinessGateEnforcesTiltFreshVisionAndAutoFailSafe() {
    ShotReadinessGate gate = new ShotReadinessGate();
    Pose3d target = new Pose3d(2.74, 4.035, 0.0, new Rotation3d());
    Pose3d onTarget = new Pose3d(2.75, 4.03, 0.0, new Rotation3d());

    // All gates satisfied
    var ok = gate.evaluate(onTarget, target, 0.05, true, 10.0, 9.95, false, 0.0);
    assertTrue(ok.readyToScore());
    assertFalse(ok.driverRumbleVisionStale());

    // Tilted chassis (> 5 deg) rejected
    Pose3d tilted = new Pose3d(2.75, 4.03, 0.0, new Rotation3d(0.0, Math.toRadians(8.0), 0.0));
    var badTilt = gate.evaluate(tilted, target, 0.05, true, 10.0, 9.95, false, 0.0);
    assertFalse(badTilt.readyToScore());

    // Stale vision in teleop -> blocks shot and signals driver rumble
    var staleTeleop = gate.evaluate(onTarget, target, 0.05, true, 10.0, 9.50, false, 0.20);
    assertFalse(staleTeleop.readyToScore());
    assertTrue(staleTeleop.driverRumbleVisionStale());

    // Stale vision in auto past 0.45s fail-safe -> releases on odometry
    var autoFailsafe = gate.evaluate(onTarget, target, 0.05, true, 10.0, 9.50, true, 0.50);
    assertTrue(autoFailsafe.readyToScore());
  }

  @Test
  void hazardAutoTuckTriggersOnlyWhenMovingTowardHazardZone() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    // blue_bump_north is [4.03..5.15, 4.63..6.48], centroid ~(4.59, 5.555)
    Translation2d approachPos = new Translation2d(3.4, 5.55);

    // Driving toward bump (+X) -> must auto-stow
    assertTrue(
        HazardAutoTuck.shouldAutoStow(
            approachPos, 2.0, 0.0, rebuilt, AllianceSide.BLUE, MatchPeriod.TELEOP, 90.0, 1.8));

    // Driving away from bump (-X) -> no auto-stow
    assertFalse(
        HazardAutoTuck.shouldAutoStow(
            approachPos, -2.0, 0.0, rebuilt, AllianceSide.BLUE, MatchPeriod.TELEOP, 90.0, 1.8));
  }

  @Test
  void collisionDetectorTriggersAfterSustainedSpeedRatioDrop() {
    CollisionDetector detector = new CollisionDetector();
    // Normal motion -> no collision
    assertFalse(detector.update(1.00, 3.0, 3.2, 4.0));
    // Stalled (ratio 0.25 <= 0.45) for < 0.15s -> not yet triggered
    assertFalse(detector.update(1.05, 0.8, 3.2, 4.0));
    assertFalse(detector.update(1.10, 0.8, 3.2, 4.0));
    // Sustained >= 0.15s -> triggers collision recovery!
    assertTrue(detector.update(1.22, 0.8, 3.2, 4.0));
    // Within 1.0 m of goal -> suppressed
    assertFalse(detector.update(1.24, 0.2, 3.2, 0.6));
  }

  @Test
  void pieceDetectionFilterAppliesDebounceHeadingConeAndMovingAverage() {
    PieceDetectionFilter filter = new PieceDetectionFilter();
    Pose2d ego = new Pose2d(2.0, 4.0, new Rotation2d()); // facing +X

    // Piece behind robot (180 deg off-axis) -> rejected by +-60 deg cone
    assertTrue(filter.update(new Translation2d(0.5, 4.0), ego, null, 1.00).isEmpty());

    // Piece ahead (+X): first sample starts 40ms debounce -> empty
    assertTrue(filter.update(new Translation2d(4.0, 4.0), ego, null, 1.00).isEmpty());
    // Sample at +50ms passes debounce and averages (4.0, 4.0) and (4.2, 4.0) -> x = 4.1
    Optional<Pose2d> smoothed = filter.update(new Translation2d(4.2, 4.0), ego, null, 1.05);
    assertTrue(smoothed.isPresent());
    assertEquals(4.1, smoothed.get().getX(), 1e-6);
  }

  @Test
  void visionPoseGateAndEgoPoseBufferEnforceRateZBoundsAndTiltAttenuation() {
    SeasonConfig rebuilt =
        SeasonConfig.fromClasspathResource("/yaece/seasons/season_2026_rebuilt.json");
    EgoPoseBuffer buf = new EgoPoseBuffer(100);
    // Spin rapidly at 8 rad/s between t=1.0 and t=1.1
    buf.addSample(new Pose3d(3.0, 4.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)), 1.00);
    buf.addSample(new Pose3d(3.0, 4.0, 0.0, new Rotation3d(0.0, 0.0, 0.8)), 1.10);
    assertEquals(8.0, buf.maxAbsYawRateInWindow(0.8, 1.2), 1e-3);

    VisionPoseGate gate = new VisionPoseGate(rebuilt);
    Pose3d validPose = new Pose3d(3.0, 4.0, 0.05, new Rotation3d());

    // Rejected due to peak yaw rate 8 rad/s > 5 rad/s
    var highSpin =
        gate.evaluate(
            validPose, 2, 2.0, 0.5, 0.05, 1.05, buf, VisionPoseGate.FusionMode.DEFAULT, false);
    assertFalse(highSpin.accepted());

    // Rejected due to Z = 1.4m > 1.0m
    Pose3d highZ = new Pose3d(3.0, 4.0, 1.4, new Rotation3d());
    var badZ =
        gate.evaluate(
            highZ, 2, 2.0, 0.5, 0.05, 2.0, null, VisionPoseGate.FusionMode.DEFAULT, false);
    assertFalse(badZ.accepted());

    // Accepted with tighter POST_BUMP_SNAP std dev than DEFAULT
    var snap =
        gate.evaluate(
            validPose,
            3,
            2.0,
            0.5,
            0.05,
            2.0,
            null,
            VisionPoseGate.FusionMode.POST_BUMP_SNAP,
            false);
    var def =
        gate.evaluate(
            validPose, 3, 2.0, 0.5, 0.05, 2.0, null, VisionPoseGate.FusionMode.DEFAULT, false);
    assertTrue(snap.accepted() && def.accepted());
    assertTrue(snap.xyStdDevMeters() < def.xyStdDevMeters());
    assertTrue(Double.isInfinite(snap.thetaStdDevRad()));

    // 6328 bump tilt odometry attenuation: 0 deg -> 1.0, 12.5 deg -> 0.5, >=25 deg -> 0.0
    assertEquals(1.0, EgoPoseBuffer.tiltOdometryScale(0.0, 0.0), 1e-6);
    assertEquals(0.5, EgoPoseBuffer.tiltOdometryScale(Math.toRadians(12.5), 0.0), 1e-3);
    assertEquals(0.0, EgoPoseBuffer.tiltOdometryScale(Math.toRadians(30.0), 0.0), 1e-6);

    // Possession watchdog factory builds cleanly
    Command watchdog =
        YaeceCommandFactory.withPossessionWatchdog(
            Command.noRequirements(co -> {}).named("Score"), () -> true, () -> 1.0, 1.5);
    assertEquals("YAECE Possession Watchdog", watchdog.name());
  }
}
