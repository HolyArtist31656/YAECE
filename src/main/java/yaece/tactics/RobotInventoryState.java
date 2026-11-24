package yaece.tactics;

import java.util.Map;
import java.util.Objects;
import yaece.season.GamePieceSpec;
import yaece.season.SeasonConfig;

/** Immutable snapshot of game pieces currently held by our robot and mechanism readiness. */
public record RobotInventoryState(
    Map<String, Integer> heldCountsByPieceId,
    boolean mechanismReadyToScore,
    boolean mechanismReadyToIntake) {

  public RobotInventoryState {
    heldCountsByPieceId = heldCountsByPieceId == null ? Map.of() : Map.copyOf(heldCountsByPieceId);
  }

  /** Empty robot inventory with both intake and scoring mechanisms ready. */
  public static RobotInventoryState empty() {
    return new RobotInventoryState(Map.of(), true, true);
  }

  /** Convenience factory for holding a single game piece of {@code pieceId}. */
  public static RobotInventoryState holding(String pieceId) {
    Objects.requireNonNull(pieceId, "pieceId must not be null");
    return new RobotInventoryState(Map.of(pieceId, 1), true, true);
  }

  /** Convenience factory for holding {@code count} pieces of {@code pieceId}. */
  public static RobotInventoryState holding(String pieceId, int count) {
    Objects.requireNonNull(pieceId, "pieceId must not be null");
    if (count <= 0) {
      return empty();
    }
    return new RobotInventoryState(Map.of(pieceId, count), true, true);
  }

  /** Returns total game pieces currently held across all types. */
  public int totalHeld() {
    int sum = 0;
    for (int c : heldCountsByPieceId.values()) {
      sum += Math.max(0, c);
    }
    return sum;
  }

  /** Returns how many pieces of {@code pieceId} are currently held. */
  public int countOf(String pieceId) {
    return Math.max(0, heldCountsByPieceId.getOrDefault(pieceId, 0));
  }

  /** Returns true if at least one game piece is currently held. */
  public boolean hasAnyHeld() {
    return totalHeld() > 0;
  }

  /** Returns true if holding another piece of {@code spec} obeys season capacity limits. */
  public boolean canHoldAnother(SeasonConfig season, GamePieceSpec spec) {
    Objects.requireNonNull(season, "season must not be null");
    Objects.requireNonNull(spec, "spec must not be null");
    return mechanismReadyToIntake
        && totalHeld() < season.maxTotalHeldGamePieces()
        && countOf(spec.id()) < spec.maxHeldSimultaneously();
  }
}
