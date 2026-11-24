package yaece.tactics;

import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;
import yaece.season.DefenseStrategyMode;

/**
 * Immutable decision emitted by {@link EncounterArbiter} on every evaluation cycle, usable both for
 * driver advisory HUDs and automated Commands v3 execution.
 */
public record TacticalRecommendation(
    EncounterIntent intent,
    Pose3d targetPose3d,
    String targetId,
    String pieceId,
    double utilityScore,
    double estimatedTimeSec,
    boolean stowMechanisms,
    DefenseStrategyMode activeDefenseStrategy,
    String reason) {

  public TacticalRecommendation {
    Objects.requireNonNull(intent, "intent must not be null");
    Objects.requireNonNull(targetPose3d, "targetPose3d must not be null");
    targetId = targetId == null ? "" : targetId;
    pieceId = pieceId == null ? "" : pieceId;
    reason = reason == null ? "" : reason;
  }

  public static TacticalRecommendation idle(Pose3d currentPose, String reason) {
    return new TacticalRecommendation(
        EncounterIntent.IDLE, currentPose, "", "", 0.0, 0.0, false, null, reason);
  }
}
