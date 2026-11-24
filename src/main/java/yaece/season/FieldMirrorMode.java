package yaece.season;

/** Defines how canonical Blue-alliance field coordinates map onto Red-alliance coordinates. */
public enum FieldMirrorMode {
  /** 180-degree rotational symmetry about {@code (L/2, W/2)}. */
  ROTATIONAL_180,
  /** Reflection across the field mid-line {@code x = L/2}. */
  REFLECTION_X,
  /** No mirroring; coordinates are returned unchanged for both alliances. */
  NONE
}
