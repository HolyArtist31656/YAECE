package yaece.console;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import yaece.ledger.OptimizationObjective;
import yaece.season.DefenseStrategyMode;

class YaeceConsoleStateTest {

  @Test
  void parsesValidEnumsAndFallsBackGracefullyOnBadInput() {
    assertEquals(
        PlayMode.DEFENSE,
        ConsoleControlState.parseOrDefault(PlayMode.class, "defense", PlayMode.OFFENSE));
    assertEquals(
        DefenseStrategyMode.ZONE_DEFENSE,
        ConsoleControlState.parseOrDefault(
            DefenseStrategyMode.class, "ZONE_DEFENSE", DefenseStrategyMode.AUTO_ADAPTIVE));
    assertEquals(
        OptimizationObjective.RANKING_POINTS,
        ConsoleControlState.parseOrDefault(
            OptimizationObjective.class, "invalid_value", OptimizationObjective.RANKING_POINTS));
  }
}
