// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.commands3;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

/**
 * Idempotent Commands v3 binding guard preventing duplicate trigger or encounter-command
 * registrations across disable/re-enable transitions in {@code ForOpMode} / {@code RobotModeScope}
 * (guarding against {@code wpilibsuite/allwpilib#9558}).
 */
public final class CommandBindingGuard {
  private final Set<String> boundKeys = new LinkedHashSet<>();

  /**
   * Executes {@code binder} only if {@code bindingKey} has not already been registered in the
   * current scope.
   *
   * @param bindingKey Unique identifier for the trigger or command binding.
   * @param binder Binding registration callback.
   * @return {@code true} if {@code binder} ran; {@code false} if skipped as a duplicate.
   */
  public boolean bindOnce(String bindingKey, Runnable binder) {
    Objects.requireNonNull(bindingKey, "bindingKey");
    Objects.requireNonNull(binder, "binder");
    if (!boundKeys.add(bindingKey)) {
      return false;
    }
    binder.run();
    return true;
  }

  /** Clears all recorded keys when a scope is explicitly torn down via {@code onExit}. */
  public void resetScope() {
    boundKeys.clear();
  }

  public boolean isBound(String bindingKey) {
    return boundKeys.contains(bindingKey);
  }

  public Set<String> getBoundKeys() {
    return Collections.unmodifiableSet(boundKeys);
  }
}
