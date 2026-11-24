package yaece.perception;

/** Coordinate frame in which an incoming sensor observation is expressed. */
public enum ObservationFrame {
  /** Pose is already expressed in canonical WPILib field coordinates (NWU meters). */
  FIELD,
  /**
   * Pose is expressed relative to the ego robot's center frame; YAECE transforms it into field
   * space using the latency-matched ego {@code Pose3d} from {@link yaece.world.EgoPoseBuffer}.
   */
  ROBOT
}
