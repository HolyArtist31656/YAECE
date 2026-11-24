package yaece.perception;

import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;

/**
 * Sensor-agnostic observation of another robot (ally, opponent, or unclassified obstacle) from
 * multi-camera bumper detection, 2D LiDAR, or fused perception.
 */
public record RobotObservation(
    Pose3d pose3d,
    ObservationFrame frame,
    RobotAffiliation affiliation,
    double confidence,
    double estimatedRadiusMeters,
    double timestampSec) {

  /** Standard nominal FRC robot bumper radius (~0.45 m for a 35x35 in bumper footprint). */
  public static final double DEFAULT_ROBOT_RADIUS_METERS = 0.45;

  public RobotObservation {
    Objects.requireNonNull(pose3d, "pose3d must not be null");
    Objects.requireNonNull(frame, "frame must not be null");
    Objects.requireNonNull(affiliation, "affiliation must not be null");
    if (!Double.isFinite(confidence) || confidence < 0.0) {
      confidence = 0.0;
    } else if (confidence > 1.0) {
      confidence = 1.0;
    }
    if (!Double.isFinite(estimatedRadiusMeters) || estimatedRadiusMeters <= 0.0) {
      throw new IllegalArgumentException("estimatedRadiusMeters must be finite and > 0");
    }
    if (!Double.isFinite(timestampSec) || timestampSec < 0.0) {
      throw new IllegalArgumentException("timestampSec must be finite and >= 0");
    }
  }

  /** Convenience factory for a field-frame robot observation with default FRC bumper radius. */
  public static RobotObservation fieldRobot(
      Pose3d fieldPose, RobotAffiliation affiliation, double confidence, double timestampSec) {
    return new RobotObservation(
        fieldPose,
        ObservationFrame.FIELD,
        affiliation,
        confidence,
        DEFAULT_ROBOT_RADIUS_METERS,
        timestampSec);
  }
}
