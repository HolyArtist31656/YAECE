package yaece.console;

import java.util.Objects;
import yaece.ledger.OptimizationObjective;
import yaece.season.DefenseStrategyMode;

/** Immutable snapshot of the live Driver Station / NT4 strategy toggles. */
public record ConsoleControlState(
    PlayMode playMode,
    DefenseStrategyMode defenseStrategyMode,
    OptimizationObjective optimizationObjective) {

  public static final ConsoleControlState DEFAULT =
      new ConsoleControlState(
          PlayMode.OFFENSE, DefenseStrategyMode.AUTO_ADAPTIVE, OptimizationObjective.MATCH_POINTS);

  public ConsoleControlState {
    Objects.requireNonNull(playMode, "playMode must not be null");
    Objects.requireNonNull(defenseStrategyMode, "defenseStrategyMode must not be null");
    Objects.requireNonNull(optimizationObjective, "optimizationObjective must not be null");
  }

  /** Safely parses an enum string from NetworkTables, returning {@code fallback} if invalid. */
  public static <E extends Enum<E>> E parseOrDefault(Class<E> enumClass, String raw, E fallback) {
    if (raw == null || raw.isBlank()) {
      return fallback;
    }
    try {
      return Enum.valueOf(enumClass, raw.trim().toUpperCase());
    } catch (IllegalArgumentException ex) {
      return fallback;
    }
  }
}
