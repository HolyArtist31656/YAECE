package yaece.commands3;

import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import yaece.tactics.TacticalRecommendation;

/**
 * Decoupled Commands v3 execution callbacks supplied by the robot project (e.g., backed by
 * YAMS/YAGSL swerve and mechanism commands) so YAECE automates encounters with zero vendor lock-in.
 *
 * @param onScore factory returning a Commands v3 command to drive and score at the recommended site
 * @param onGather factory returning a Commands v3 command to drive and intake the recommended piece
 * @param onHoldStage factory returning a Commands v3 command to drive to and wait at the hold pose
 * @param onDefend factory returning a Commands v3 command to drive to the defensive/retreat pose
 * @param onStowMechanisms optional command supplier to stow mechanisms inside the frame perimeter
 */
public record YaeceExecutionHooks(
    Function<TacticalRecommendation, Command> onScore,
    Function<TacticalRecommendation, Command> onGather,
    Function<TacticalRecommendation, Command> onHoldStage,
    Function<TacticalRecommendation, Command> onDefend,
    Supplier<Command> onStowMechanisms) {

  public YaeceExecutionHooks {
    Objects.requireNonNull(onScore, "onScore hook must not be null");
    Objects.requireNonNull(onGather, "onGather hook must not be null");
    Objects.requireNonNull(onHoldStage, "onHoldStage hook must not be null");
    Objects.requireNonNull(onDefend, "onDefend hook must not be null");
  }
}
