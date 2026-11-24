package yaece.world;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import org.wpilib.math.geometry.Translation2d;
import yaece.perception.RobotAffiliation;
import yaece.season.PinRuleSpec;

/**
 * Enforces FRC evergreen pinning rules (3.0 s max pin, 2.2 s proactive backoff trigger, 1.83 m / 6
 * ft separation pause, and 3.2 s cooldown hold reset) per tracked opponent.
 *
 * <p>If ego re-enters inside 1.83 m before the cooldown completes, the pin count resumes from its
 * paused value rather than resetting—matching referee rule semantics exactly.
 */
public final class PinGuardTracker {
  private static final double CONTACT_MARGIN_METERS = 0.22;
  private static final double STALLED_SPEED_MPS = 0.35;

  private static final class MutablePin {
    PinStatus status = PinStatus.FREE;
    double activePinSec = 0.0;
    double cooldownSec = 0.0;
    double lastSeparationMeters = 0.0;
  }

  private final PinRuleSpec rules;
  private final double egoBumperRadiusMeters;
  private final Map<Integer, MutablePin> states = new HashMap<>();

  public PinGuardTracker(PinRuleSpec rules, double egoBumperRadiusMeters) {
    this.rules = Objects.requireNonNull(rules, "rules must not be null");
    this.egoBumperRadiusMeters = egoBumperRadiusMeters;
  }

  /** Updates per-opponent pin timers for one cycle of length {@code dtSec}. */
  public Map<Integer, OpponentPinState> update(
      Translation2d egoPosition, List<TrackedRobot> trackedRobots, double dtSec) {
    double dt = Math.max(0.0, dtSec);
    Set<Integer> seenOpponents = new HashSet<>();
    Map<Integer, OpponentPinState> snapshot = new HashMap<>();

    for (TrackedRobot bot : trackedRobots) {
      if (bot.affiliation() != RobotAffiliation.OPPONENT) {
        continue;
      }
      int id = bot.trackId();
      seenOpponents.add(id);
      MutablePin st = states.computeIfAbsent(id, k -> new MutablePin());

      double centerDist = egoPosition.getDistance(bot.pose3d().getTranslation().toTranslation2d());
      double bumperGap =
          Math.max(0.0, centerDist - egoBumperRadiusMeters - bot.estimatedRadiusMeters());
      st.lastSeparationMeters = bumperGap;

      boolean inContactAndStalled =
          bumperGap <= CONTACT_MARGIN_METERS && bot.speedMps() <= STALLED_SPEED_MPS;
      boolean separatedBeyondRule = bumperGap >= rules.separationDistanceMeters();

      switch (st.status) {
        case FREE -> {
          if (inContactAndStalled) {
            st.status = PinStatus.PINNING_ACTIVE;
            st.activePinSec = dt;
            st.cooldownSec = 0.0;
          }
        }
        case PINNING_ACTIVE -> {
          if (separatedBeyondRule) {
            st.status = PinStatus.COOLDOWN_PAUSED;
            st.cooldownSec = dt;
          } else {
            // Count continues while within 6 ft (even if briefly oscillating)
            st.activePinSec += dt;
          }
        }
        case COOLDOWN_PAUSED -> {
          if (separatedBeyondRule) {
            st.cooldownSec += dt;
            if (st.cooldownSec >= rules.cooldownHoldSeconds()) {
              st.status = PinStatus.FREE;
              st.activePinSec = 0.0;
              st.cooldownSec = 0.0;
            }
          } else {
            // Re-entered inside 6 ft before cooldown finished -> resume paused pin count!
            st.status = PinStatus.PINNING_ACTIVE;
            st.cooldownSec = 0.0;
            st.activePinSec += dt;
          }
        }
      }

      boolean backoff =
          (st.status == PinStatus.PINNING_ACTIVE
                  && st.activePinSec >= rules.backoffTriggerSeconds())
              || (st.status == PinStatus.COOLDOWN_PAUSED
                  && st.activePinSec >= rules.backoffTriggerSeconds());

      snapshot.put(
          id,
          new OpponentPinState(id, st.status, st.activePinSec, st.cooldownSec, backoff, bumperGap));
    }

    // Prune tracks that no longer exist in the world model
    states.keySet().removeIf(id -> !seenOpponents.contains(id));
    return Collections.unmodifiableMap(snapshot);
  }

  public void reset() {
    states.clear();
  }
}
