package yaece.commands3;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import yaece.tactics.EncounterIntent;
import yaece.tactics.TacticalRecommendation;
import yaece.tactics.YaeceEngine;

/**
 * Builds idiomatic WPILib 2027 Commands v3 coroutine routines that execute {@link
 * YaeceEngine#getLatestRecommendation()} automatically while guaranteeing instant single-tick
 * driver override/cancellation.
 */
public final class YaeceCommandFactory {
  private YaeceCommandFactory() {}

  /**
   * Creates a Commands v3 coordinating coroutine that monitors {@link
   * YaeceEngine#getLatestRecommendation()} each scheduler cycle, forks the appropriate child
   * command whenever intent/target changes, and exits immediately (canceling all forked children)
   * the instant {@code driverOverride.getAsBoolean()} returns {@code true}.
   */
  public static Command runAutonomousEncounter(
      YaeceEngine engine, YaeceExecutionHooks hooks, BooleanSupplier driverOverride) {
    Objects.requireNonNull(engine, "engine must not be null");
    Objects.requireNonNull(hooks, "hooks must not be null");
    Objects.requireNonNull(driverOverride, "driverOverride must not be null");

    return Command.noRequirements(
            co -> {
              EncounterIntent activeIntent = EncounterIntent.IDLE;
              String activeTargetId = "";

              while (!driverOverride.getAsBoolean()) {
                TacticalRecommendation rec = engine.getLatestRecommendation();
                boolean changed =
                    rec.intent() != activeIntent || !rec.targetId().equals(activeTargetId);

                if (changed) {
                  activeIntent = rec.intent();
                  activeTargetId = rec.targetId();

                  if (rec.stowMechanisms() && hooks.onStowMechanisms() != null) {
                    co.fork(hooks.onStowMechanisms().get());
                  }

                  Command child =
                      switch (rec.intent()) {
                        case SCORE -> hooks.onScore().apply(rec);
                        case GATHER -> hooks.onGather().apply(rec);
                        case HOLD_STAGE -> hooks.onHoldStage().apply(rec);
                        case DEFEND_INTERCEPT, DEFEND_PIN_RETREAT -> hooks.onDefend().apply(rec);
                        case IDLE -> null;
                      };
                  if (child != null) {
                    co.fork(child);
                  }
                }
                co.yield();
              }
            })
        .named("YAECE Autonomous Encounter");
  }
}
