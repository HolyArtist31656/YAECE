package yaece.console;

/** Top-level tactical mode toggleable on the fly during a match. */
public enum PlayMode {
  /** Actively gather game pieces and score to maximize PPS or Ranking Points. */
  OFFENSE,
  /** Intercept and block opposing robots while enforcing FRC pin and restricted-zone rules. */
  DEFENSE
}
