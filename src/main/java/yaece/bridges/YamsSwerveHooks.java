// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.bridges;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import yaece.commands3.YaeceCommandFactory;
import yaece.commands3.YaeceExecutionHooks;
import yaece.tactics.TacticalRecommendation;
import yaece.tactics.YaeceEngine;

/**
 * Simplified builder wiring YAMS/YAGSL {@code SwerveDrive.driveToPose(Pose2d)} and continuous
 * {@code Supplier<Pose2d>} tracking commands into {@link YaeceExecutionHooks} and {@link
 * YaeceCommandFactory}.
 */
public final class YamsSwerveHooks {
  private YamsSwerveHooks() {}

  /**
   * Creates {@link YaeceExecutionHooks} from a single {@code driveToPose} factory (e.g., {@code
   * swerve::driveToPose}) plus optional intake, scoring, and stow mechanism command factories.
   */
  public static YaeceExecutionHooks create(
      Function<Pose2d, Command> driveToPose,
      Function<TacticalRecommendation, Command> scoreSequence,
      Function<TacticalRecommendation, Command> gatherSequence,
      Supplier<Command> stowMechanisms) {
    Objects.requireNonNull(driveToPose, "driveToPose must not be null");

    Function<TacticalRecommendation, Command> onScore =
        scoreSequence != null
            ? scoreSequence
            : rec -> driveToPose.apply(rec.targetPose3d().toPose2d());
    Function<TacticalRecommendation, Command> onGather =
        gatherSequence != null
            ? gatherSequence
            : rec -> driveToPose.apply(rec.targetPose3d().toPose2d());
    Function<TacticalRecommendation, Command> onHold =
        rec -> driveToPose.apply(rec.targetPose3d().toPose2d());
    Function<TacticalRecommendation, Command> onDefend =
        rec -> driveToPose.apply(rec.targetPose3d().toPose2d());

    return new YaeceExecutionHooks(onScore, onGather, onHold, onDefend, stowMechanisms);
  }

  /**
   * Creates a continuous Commands v3 encounter command backed by a live {@code Supplier<Pose2d>}
   * drive controller (recommended for moving opponent defense and rolling game pieces).
   */
  public static Command createContinuous(
      YaeceEngine engine,
      Function<Supplier<Pose2d>, Command> continuousDriveToPose,
      Supplier<Command> stowMechanisms,
      BooleanSupplier driverOverride) {
    return YaeceCommandFactory.runContinuousEncounter(
        engine, continuousDriveToPose, stowMechanisms, driverOverride);
  }
}
