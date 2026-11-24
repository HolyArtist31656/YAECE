package yaece.bridges;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.season.SeasonConfig;
import yaece.tactics.YaeceEngine;

class QuestNavAdapterTest {

  @Test
  void appliesInverseMountTransformWhenTrackingAndFallsBackToOdometryOnLoss() {
    SeasonConfig season =
        SeasonConfig.fromClasspathResource("/yaece/seasons/sample_2027_template.json");
    YaeceEngine engine = YaeceEngine.builder(season).build();

    // Quest mounted 0.3m forward, 0.5m above robot center
    Transform3d robotToQuest = new Transform3d(new Translation3d(0.3, 0.0, 0.5), new Rotation3d());
    Pose3d fallbackOdom = new Pose3d(new Translation3d(1.0, 2.0, 0.0), new Rotation3d());
    QuestNavAdapter adapter = new QuestNavAdapter(robotToQuest, () -> fallbackOdom);

    // Tracking OK: quest at (4.3, 3.0, 0.5) -> robot center at (4.0, 3.0, 0.0)
    Pose3d questPose = new Pose3d(new Translation3d(4.3, 3.0, 0.5), new Rotation3d());
    Pose3d robotCenter = adapter.feedFrame(engine, questPose, 5.0, true);
    assertEquals(4.0, robotCenter.getX(), 1e-6);
    assertEquals(3.0, robotCenter.getY(), 1e-6);
    assertEquals(0.0, robotCenter.getZ(), 1e-6);

    // Tracking lost: seamlessly falls back to odometry
    Pose3d fallbackUsed = adapter.feedFrame(engine, questPose, 5.02, false);
    assertEquals(fallbackOdom, fallbackUsed);
  }
}
