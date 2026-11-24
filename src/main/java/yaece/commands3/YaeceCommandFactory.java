package yaece.commands3;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import yaece.tactics.EncounterIntent;
import yaece.tactics.TacticalRecommendation;
import yaece.tactics.YaeceEngine;

/**
 * Builds idiomatic WPILib 2027 Commands v3 coroutine routines that execute {@link
 * YaeceEngine#getLatestRecommendation()} automatically while guaranteeing:
 *
 * <ul>
 *   <li>Instant single-tick driver joystick preemption,
 *   <li>Continuous tracking of moving targets via generation-guarded cancellation or {@code
 *       Supplier<Pose2d>} streaming,
 *   <li>{@code 2.0 m} terminal lockout window on static scoring approaches (preventing terminal
 *       replan hunting, inspired by PathPlanner {@code PathfindingCommand}),
 *   <li>Optional piece-possession watchdog for stolen/dropped game-piece recovery (FRC 254).
 * </ul>
 */
public final class YaeceCommandFactory {
  private static final double POSE_RETARGET_THRESHOLD_METERS = 0.20;
  private static final double YAW_RETARGET_THRESHOLD_RAD = 0.20;
  private static final double TERMINAL_LOCKOUT_METERS = 2.0;

  private YaeceCommandFactory() {}

  public static Command runAutonomousEncounter(
      YaeceEngine engine, YaeceExecutionHooks hooks, BooleanSupplier driverOverride) {
    Objects.requireNonNull(engine, "engine must not be null");
    Objects.requireNonNull(hooks, "hooks must not be null");
    Objects.requireNonNull(driverOverride, "driverOverride must not be null");

    return Command.noRequirements(
            co -> {
              EncounterIntent activeIntent = EncounterIntent.IDLE;
              String activeTargetId = "";
              Pose2d activePose = new Pose2d();
              AtomicInteger generation = new AtomicInteger(0);

              while (!driverOverride.getAsBoolean()) {
                TacticalRecommendation rec = engine.getLatestRecommendation();
                Pose2d recPose = rec.targetPose3d().toPose2d();
                Pose2d egoPose = engine.getWorldSnapshot().egoPose3d().toPose2d();
                double distToGoal = egoPose.getTranslation().getDistance(recPose.getTranslation());

                boolean isDefensive =
                    rec.intent() == EncounterIntent.DEFEND_INTERCEPT
                        || rec.intent() == EncounterIntent.DEFEND_PIN_RETREAT;
                boolean inTerminalLockout =
                    !isDefensive
                        && rec.intent() == activeIntent
                        && rec.targetId().equals(activeTargetId)
                        && distToGoal < TERMINAL_LOCKOUT_METERS;

                boolean poseShifted =
                    !inTerminalLockout
                        && (recPose.getTranslation().getDistance(activePose.getTranslation())
                                > POSE_RETARGET_THRESHOLD_METERS
                            || Math.abs(
                                    wrapRad(
                                        recPose.getRotation().getRadians()
                                            - activePose.getRotation().getRadians()))
                                > YAW_RETARGET_THRESHOLD_RAD);
                boolean changed =
                    rec.intent() != activeIntent
                        || !rec.targetId().equals(activeTargetId)
                        || (rec.intent() != EncounterIntent.IDLE && poseShifted);

                if (changed) {
                  int nextGen = generation.incrementAndGet();
                  activeIntent = rec.intent();
                  activeTargetId = rec.targetId();
                  activePose = recPose;

                  if (rec.stowMechanisms() && hooks.onStowMechanisms() != null) {
                    co.fork(hooks.onStowMechanisms().get());
                  }

                  Command rawChild =
                      switch (rec.intent()) {
                        case SCORE -> hooks.onScore().apply(rec);
                        case GATHER -> hooks.onGather().apply(rec);
                        case HOLD_STAGE -> hooks.onHoldStage().apply(rec);
                        case DEFEND_INTERCEPT, DEFEND_PIN_RETREAT -> hooks.onDefend().apply(rec);
                        case IDLE -> null;
                      };
                  if (rawChild != null) {
                    Command guarded =
                        rawChild
                            .until(
                                () -> generation.get() != nextGen || driverOverride.getAsBoolean())
                            .named("YAECE Guarded Action");
                    co.fork(guarded);
                  }
                }
                co.yield();
              }
              generation.incrementAndGet();
            })
        .named("YAECE Autonomous Encounter");
  }

  /**
   * Runs a continuous {@code Supplier<Pose2d>} tracking command reading {@link
   * YaeceEngine#getLatestRecommendation()} every 20 ms without restarting the drive command.
   */
  public static Command runContinuousEncounter(
      YaeceEngine engine,
      Function<Supplier<Pose2d>, Command> continuousDriveFactory,
      Supplier<Command> stowMechanisms,
      BooleanSupplier driverOverride) {
    Objects.requireNonNull(engine, "engine must not be null");
    Objects.requireNonNull(continuousDriveFactory, "continuousDriveFactory must not be null");
    Objects.requireNonNull(driverOverride, "driverOverride must not be null");

    Supplier<Pose2d> livePoseSupplier =
        () -> engine.getLatestRecommendation().targetPose3d().toPose2d();

    return Command.noRequirements(
            co -> {
              boolean wasStowed = false;
              Command driveCmd =
                  continuousDriveFactory
                      .apply(livePoseSupplier)
                      .until(driverOverride)
                      .named("YAECE Continuous Drive");
              co.fork(driveCmd);

              while (!driverOverride.getAsBoolean()) {
                TacticalRecommendation rec = engine.getLatestRecommendation();
                if (rec.stowMechanisms() && !wasStowed && stowMechanisms != null) {
                  co.fork(stowMechanisms.get());
                }
                wasStowed = rec.stowMechanisms();
                co.yield();
              }
            })
        .named("YAECE Continuous Encounter");
  }

  /**
   * Wraps a transit-to-score command with a piece-possession watchdog (inspired by FRC 254's {@code
   * 1.5 s} retry guard): if {@code hasGamePiece} stays {@code false} for {@code missingTimeoutSec},
   * cancels the wrapped command so the encounter engine immediately re-targets.
   */
  public static Command withPossessionWatchdog(
      Command scoreCommand,
      BooleanSupplier hasGamePiece,
      Supplier<Double> fpgaTimeSecSupplier,
      double missingTimeoutSec) {
    Objects.requireNonNull(scoreCommand, "scoreCommand must not be null");
    Objects.requireNonNull(hasGamePiece, "hasGamePiece must not be null");
    Objects.requireNonNull(fpgaTimeSecSupplier, "fpgaTimeSecSupplier must not be null");

    return Command.noRequirements(
            co -> {
              double missingStartSec = -1.0;
              Command child = scoreCommand; // lifecycle controlled by parent coroutine scope
              co.fork(child);
              while (true) {
                if (hasGamePiece.getAsBoolean()) {
                  missingStartSec = -1.0;
                } else {
                  double now = fpgaTimeSecSupplier.get();
                  if (missingStartSec < 0.0) {
                    missingStartSec = now;
                  } else if ((now - missingStartSec) >= missingTimeoutSec) {
                    return; // exits scope and cancels forked child
                  }
                }
                co.yield();
              }
            })
        .named("YAECE Possession Watchdog");
  }

  private static double wrapRad(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
