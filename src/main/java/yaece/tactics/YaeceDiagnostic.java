package yaece.tactics;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import yaece.commands3.YaeceExecutionHooks;
import yaece.season.ScoringIntervalSpec;
import yaece.season.ScoringSiteSpec;
import yaece.season.SeasonConfig;

/**
 * Startup validation and diagnostic report utility (mirroring {@code YacSimDiagnostic}) so teams
 * catch misnamed {@link SeasonConfig} keys, unreferenced scoring intervals, or missing hooks at
 * robot boot rather than during a match.
 */
public final class YaeceDiagnostic {
  private YaeceDiagnostic() {}

  public record DiagnosticReport(
      boolean healthy, List<String> warnings, List<String> errors, String formattedReport) {}

  public static DiagnosticReport generateReport(SeasonConfig season, YaeceEngine engine) {
    return generateReport(season, engine, null);
  }

  public static DiagnosticReport generateReport(
      SeasonConfig season, YaeceEngine engine, YaeceExecutionHooks hooks) {
    Objects.requireNonNull(season, "season must not be null");
    List<String> warnings = new ArrayList<>();
    List<String> errors = new ArrayList<>();

    if (season.gamePieces().isEmpty()) {
      errors.add("SeasonConfig '" + season.seasonName() + "' defines 0 gamePieces.");
    }
    if (season.scoringSites().isEmpty()) {
      errors.add("SeasonConfig '" + season.seasonName() + "' defines 0 scoringSites.");
    }

    Set<String> dynamicKeys = new LinkedHashSet<>();
    for (ScoringSiteSpec site : season.scoringSites()) {
      if (!site.dynamicAvailabilityKey().isEmpty()) {
        dynamicKeys.add(site.dynamicAvailabilityKey());
      }
      if (site.autoPoints() == 0.0 && site.teleopPoints() == 0.0) {
        warnings.add("ScoringSite '" + site.id() + "' has 0.0 autoPoints and 0.0 teleopPoints.");
      }
    }

    for (ScoringIntervalSpec interval : season.scoringIntervals()) {
      boolean matchedAny = false;
      for (ScoringSiteSpec site : season.scoringSites()) {
        if (interval.matchesSite(site)) {
          matchedAny = true;
          break;
        }
      }
      if (!matchedAny) {
        warnings.add(
            "ScoringInterval '"
                + interval.id()
                + "' does not match any ScoringSiteSpec by ID or tag.");
      }
    }

    if (hooks != null && hooks.onStowMechanisms() == null) {
      warnings.add("YaeceExecutionHooks.onStowMechanisms() is null; defense mode will not stow.");
    }

    StringBuilder sb = new StringBuilder();
    sb.append("=== YAECE Startup Diagnostic (")
        .append(season.seasonName())
        .append(") ===\n")
        .append("  Field: ")
        .append(
            String.format("%.2fm x %.2fm", season.fieldLengthMeters(), season.fieldWidthMeters()))
        .append(" | Mirror: ")
        .append(season.mirrorMode())
        .append("\n  GamePieces: ")
        .append(season.gamePieces().size())
        .append(" | ScoringSites: ")
        .append(season.scoringSites().size())
        .append(" | StaticObstacles: ")
        .append(season.staticObstacles().size())
        .append(" | RestrictedZones: ")
        .append(season.restrictedZones().size())
        .append("\n  ScoringIntervals: ")
        .append(season.scoringIntervals().size())
        .append(" | DynamicKeys: ")
        .append(dynamicKeys.isEmpty() ? "none" : String.join(", ", dynamicKeys))
        .append("\n  Engine: ")
        .append(engine != null ? "attached" : "not attached")
        .append(" | Hooks: ")
        .append(hooks != null ? "attached" : "advisory-only")
        .append("\n  Errors: ")
        .append(errors.size())
        .append(" | Warnings: ")
        .append(warnings.size());
    for (String e : errors) {
      sb.append("\n  [ERROR] ").append(e);
    }
    for (String w : warnings) {
      sb.append("\n  [WARN]  ").append(w);
    }

    return new DiagnosticReport(
        errors.isEmpty(), List.copyOf(warnings), List.copyOf(errors), sb.toString());
  }

  public static DiagnosticReport printReport(
      SeasonConfig season, YaeceEngine engine, YaeceExecutionHooks hooks) {
    DiagnosticReport report = generateReport(season, engine, hooks);
    System.out.println(report.formattedReport());
    return report;
  }
}
