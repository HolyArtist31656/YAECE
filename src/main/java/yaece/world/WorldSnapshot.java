package yaece.world;

import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose3d;
import yaece.perception.RobotAffiliation;

/** Immutable, thread-safe snapshot of the entire 3D FRC field world model at a single timestamp. */
public record WorldSnapshot(
    double timestampSec,
    Pose3d egoPose3d,
    double egoVxMps,
    double egoVyMps,
    List<TrackedGamePiece> gamePieces,
    List<TrackedRobot> robots,
    Map<Integer, OpponentPinState> pinStates) {

  public static final WorldSnapshot EMPTY =
      new WorldSnapshot(0.0, new Pose3d(), 0.0, 0.0, List.of(), List.of(), Map.of());

  /** Returns all tracked opposing alliance robots. */
  public List<TrackedRobot> opponents() {
    return robots.stream().filter(r -> r.affiliation() == RobotAffiliation.OPPONENT).toList();
  }

  /** Returns all tracked alliance partner robots. */
  public List<TrackedRobot> allies() {
    return robots.stream().filter(r -> r.affiliation() == RobotAffiliation.ALLY).toList();
  }

  /** Returns true if any opponent pin timer has reached the proactive backoff threshold. */
  public boolean anyPinBackoffRequired() {
    for (OpponentPinState st : pinStates.values()) {
      if (st.backoffRequired()) {
        return true;
      }
    }
    return false;
  }
}
