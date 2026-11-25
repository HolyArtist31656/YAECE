package yaece.util;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Wall-clock startup watchdog backed strictly by {@link System#nanoTime()} rather than WPILib's
 * {@code Timer} (which remains frozen during robot construction / initialization on SystemCore).
 */
public final class InitWallTimeWatchdog {
  private final long budgetNanos;
  private final long startNanos;
  private long lastMarkNanos;
  private final Map<String, Double> phaseMillis = new LinkedHashMap<>();

  public InitWallTimeWatchdog(double budgetMillis) {
    if (budgetMillis <= 0.0 || !Double.isFinite(budgetMillis)) {
      throw new IllegalArgumentException("budgetMillis must be positive and finite.");
    }
    this.budgetNanos = Math.round(budgetMillis * 1_000_000.0);
    this.startNanos = System.nanoTime();
    this.lastMarkNanos = this.startNanos;
  }

  /** Records the elapsed wall-clock milliseconds since the previous phase mark. */
  public double markPhase(String phaseName) {
    Objects.requireNonNull(phaseName, "phaseName");
    long now = System.nanoTime();
    double ms = (now - lastMarkNanos) / 1_000_000.0;
    phaseMillis.put(phaseName, ms);
    lastMarkNanos = now;
    return ms;
  }

  public double getTotalElapsedMillis() {
    return (System.nanoTime() - startNanos) / 1_000_000.0;
  }

  public boolean isOverBudget() {
    return (System.nanoTime() - startNanos) > budgetNanos;
  }

  public Map<String, Double> getPhaseMillis() {
    return Collections.unmodifiableMap(phaseMillis);
  }
}
