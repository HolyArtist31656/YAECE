package yaece.season;

/** Enforcement semantics for a field restricted or protected zone. */
public enum ZoneRuleType {
  /** Ego chassis bumper footprint may never intersect this polygon while active. */
  HARD_NO_ENTRY,
  /** Ego robot may not contact any opponent partially or fully inside this polygon while active. */
  NO_OPPONENT_CONTACT,
  /** Legal to traverse, but penalized in pathfinding and defensive positioning. */
  SOFT_AVOIDANCE
}
