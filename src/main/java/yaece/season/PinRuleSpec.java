// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

/**
 * Season-configurable FRC pinning rule parameters.
 *
 * <p>Defaults match current evergreen FRC rules (G425 / G418): 3.0 s maximum pin duration, 2.2 s
 * proactive backoff trigger, 6 ft (~1.83 m) separation threshold, and 3.2 s cooldown hold to ensure
 * a clean referee count reset.
 */
public record PinRuleSpec(
    double maxPinSeconds,
    double backoffTriggerSeconds,
    double separationDistanceMeters,
    double cooldownHoldSeconds) {

  /** Standard 2025/2026+ FRC evergreen pinning parameters. */
  public static final PinRuleSpec FRC_DEFAULT = new PinRuleSpec(3.0, 2.2, 1.83, 3.2);

  public PinRuleSpec {
    if (!Double.isFinite(maxPinSeconds) || maxPinSeconds <= 0.0) {
      throw new IllegalArgumentException("PinRuleSpec.maxPinSeconds must be > 0");
    }
    if (!Double.isFinite(backoffTriggerSeconds)
        || backoffTriggerSeconds <= 0.0
        || backoffTriggerSeconds >= maxPinSeconds) {
      throw new IllegalArgumentException(
          "PinRuleSpec.backoffTriggerSeconds must be in (0, maxPinSeconds)");
    }
    if (!Double.isFinite(separationDistanceMeters) || separationDistanceMeters <= 0.0) {
      throw new IllegalArgumentException("PinRuleSpec.separationDistanceMeters must be > 0");
    }
    if (!Double.isFinite(cooldownHoldSeconds) || cooldownHoldSeconds <= 0.0) {
      throw new IllegalArgumentException("PinRuleSpec.cooldownHoldSeconds must be > 0");
    }
  }
}
