package yaece.perception;

/** Alliance affiliation of an observed non-ego robot on the field. */
public enum RobotAffiliation {
  /** Alliance partner robot (same bumper color as ego). */
  ALLY,
  /** Opposing alliance robot (opposite bumper color). */
  OPPONENT,
  /** Unclassified robot obstacle (e.g., 2D LiDAR cluster without camera color match). */
  UNKNOWN
}
