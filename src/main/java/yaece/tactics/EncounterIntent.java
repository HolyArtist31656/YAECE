package yaece.tactics;

/** High-level tactical action recommended or executed by YAECE. */
public enum EncounterIntent {
  /** Drive to a scoring site and score an already-held game piece. */
  SCORE,
  /** Drive to and acquire an uncontested 6-DoF game piece on the field. */
  GATHER,
  /** Move to a staging/waiting pose while scoring lanes clear or dynamic sites activate. */
  HOLD_STAGE,
  /** Intercept and block an opposing robot along its predicted scoring path or defense corridor. */
  DEFEND_INTERCEPT,
  /** Proactively retreat >= 1.83 m to clear an active FRC 3-second pin count. */
  DEFEND_PIN_RETREAT,
  /** No viable offensive or defensive target currently exists. */
  IDLE
}
