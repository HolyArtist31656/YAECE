package yaece.season;

import java.util.List;

/** Exception thrown when a YAECE season JSON file fails to load or fails semantic validation. */
public final class SeasonConfigException extends RuntimeException {
  private final List<String> violations;

  public SeasonConfigException(String message) {
    super(message);
    this.violations = List.of(message);
  }

  public SeasonConfigException(String message, Throwable cause) {
    super(message, cause);
    this.violations = List.of(message);
  }

  public SeasonConfigException(List<String> violations) {
    super(
        "Invalid YAECE SeasonConfig ("
            + violations.size()
            + " errors): "
            + String.join("; ", violations));
    this.violations = List.copyOf(violations);
  }

  public List<String> getViolations() {
    return violations;
  }
}
