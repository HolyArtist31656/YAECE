package yaece.season;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

class FieldMirroringTest {
  private static final double FIELD_L = 16.54;
  private static final double FIELD_W = 8.07;
  private static final double EPS = 1e-9;

  @Test
  void rotational180MirrorsPose3dAndIsInvolutive() {
    Pose3d blue =
        new Pose3d(
            new Translation3d(2.0, 1.5, 1.45), new Rotation3d(0.1, -0.2, Math.toRadians(30)));
    Pose3d red =
        FieldMirroring.mirror(
            blue, AllianceSide.RED, FIELD_L, FIELD_W, FieldMirrorMode.ROTATIONAL_180);

    assertEquals(FIELD_L - 2.0, red.getX(), EPS);
    assertEquals(FIELD_W - 1.5, red.getY(), EPS);
    assertEquals(1.45, red.getZ(), EPS);

    Pose3d roundTrip =
        FieldMirroring.mirror(
            red, AllianceSide.RED, FIELD_L, FIELD_W, FieldMirrorMode.ROTATIONAL_180);
    assertEquals(blue.getX(), roundTrip.getX(), EPS);
    assertEquals(blue.getY(), roundTrip.getY(), EPS);
    assertEquals(blue.getZ(), roundTrip.getZ(), EPS);
    assertEquals(blue.getRotation().getX(), roundTrip.getRotation().getX(), 1e-6);
    assertEquals(blue.getRotation().getY(), roundTrip.getRotation().getY(), 1e-6);
    assertEquals(blue.getRotation().getZ(), roundTrip.getRotation().getZ(), 1e-6);
  }

  @Test
  void reflectionXMirrorsPose2dAndPreservesY() {
    Pose2d blue = new Pose2d(new Translation2d(3.0, 5.5), Rotation2d.fromDegrees(45));
    Pose2d red =
        FieldMirroring.mirror(
            blue, AllianceSide.RED, FIELD_L, FIELD_W, FieldMirrorMode.REFLECTION_X);

    assertEquals(FIELD_L - 3.0, red.getX(), EPS);
    assertEquals(5.5, red.getY(), EPS);
    assertEquals(135.0, red.getRotation().getDegrees(), 1e-6);

    Pose2d roundTrip =
        FieldMirroring.mirror(
            red, AllianceSide.RED, FIELD_L, FIELD_W, FieldMirrorMode.REFLECTION_X);
    assertEquals(blue.getX(), roundTrip.getX(), EPS);
    assertEquals(blue.getY(), roundTrip.getY(), EPS);
    assertEquals(blue.getRotation().getDegrees(), roundTrip.getRotation().getDegrees(), 1e-6);
  }

  @Test
  void blueAllianceOrNoneModeLeavesPoseUnchanged() {
    Pose3d blue = new Pose3d(new Translation3d(4.0, 2.0, 0.5), new Rotation3d(0.0, 0.0, 1.0));
    assertEquals(
        blue,
        FieldMirroring.mirror(
            blue, AllianceSide.BLUE, FIELD_L, FIELD_W, FieldMirrorMode.ROTATIONAL_180));
    assertEquals(
        blue,
        FieldMirroring.mirror(blue, AllianceSide.RED, FIELD_L, FIELD_W, FieldMirrorMode.NONE));
  }
}
