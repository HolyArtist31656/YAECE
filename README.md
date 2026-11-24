# YAECE — Yet Another Encounter Control Engine

**Yet Another Encounter Control Engine (YAECE)** is a game-agnostic, strictly **sensor-agnostic**, zero-dependency FRC Java 25 vendordep for **WPILib 2027 (`2027.0.0-alpha-7`)** and **Commands v3**. No specific camera, LiDAR, or vision library is required—teams feed standardized `Pose3d` / 6-DoF observations from any source, while optional bridges integrate QuestNav, PhotonVision, LiDAR, and YAMS/YAGSL.

## Locked 2027 Stack (Mirrors YACSim)

| Property | Value |
| --- | --- |
| WPILib Version | `2027.0.0-alpha-7` (`2027_alpha7`, `org.wpilib.*`) |
| Java Toolchain | Java 25 (`C:\Users\Public\wpilib\2027_alpha7\jdk`) |
| Gradle | `9.4.1` |
| Command Framework | Commands v3 only (`org.wpilib.command3`) |
| Third-Party Runtime JARs | **0** (internal `MinimalJson` parser + WPILib/WPIMath only) |
| Formatting | Spotless (`googleJavaFormat`) |
| Target Hardware | 2 GB RAM Limelight SystemCore (CM5 Cortex-A76) |

## Phase 1 (`v2027.0.1-alpha1`) Contents

1. **Vendordep & GitHub Pages Distribution**: `build.gradle`, `gradle.properties`, `vendordeps/yaece_commands3.json`, and `.github/workflows/publish-pages.yml`.
2. **Zero-Dependency JSON Engine (`yaece.json`)**: Strict RFC 8259 `MinimalJson` parser with line/column diagnostics.
3. **Extended Season Configuration (`yaece.season`)**: `SeasonConfig` with `autoPoints`/`teleopPoints`, site `tags`, `pinRules` (FRC 3 s / 1.83 m), `restrictedZones` (`HARD_NO_ENTRY`, `NO_OPPONENT_CONTACT`, `SOFT_AVOIDANCE` with Endgame time windows), `scoringObjectives` (`BONUS_MATCH_POINTS` & `RANKING_POINT`), `defenseDefaults` (`HIGHEST_THREAT`, `ZONE_DEFENSE`, `AUTO_ADAPTIVE` + named defense corridors), and `FieldMirroring`.
4. **Live Alliance & Opponent Score Ledger (`yaece.ledger`)**: `MatchScoreLedger` reconciling self-tracked timestamped scores with optional timestamped Driver Station console inputs (teammate contributions, observed alliance score/pieces, opponent score/pieces), exponentially-weighted opponent score extrapolation (`ScoreProjection`), `OptimizationObjective` (`MATCH_POINTS` vs. `RANKING_POINTS`), and score-aware `DefenseModeResolver`.
5. **JUnit 5 Verification Suite**: 4 test classes covering JSON parsing, 3D/2D mirroring, extended SeasonConfig validation, timestamped score reconciliation, opponent extrapolation, and adaptive defense switching.

## Quick Start

```java
import java.nio.file.Path;
import yaece.ledger.DefenseModeResolver;
import yaece.ledger.MatchScoreLedger;
import yaece.ledger.ScoreProjection;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;
import yaece.season.SeasonConfig;

SeasonConfig season = SeasonConfig.fromFile(Path.of("src/main/deploy/yaece/season_2027.json"));
MatchScoreLedger ledger = new MatchScoreLedger(season);

// Track our own robot's scored pieces with elapsed match timestamps
ledger.recordSelfScored("high_post_left", "cylinder_core", MatchPeriod.AUTO, 6.0);

// Optional timestamped Driver Station console inputs (fused without double-counting)
ledger.updateObservedAllianceScore(24.0, 20.0);
ledger.updateOpponentObservedScore(18.0, 20.0);
ledger.updateOpponentObservedScore(48.0, 50.0);

ScoreProjection proj = ledger.projectScores(55.0);
DefenseStrategyMode activeDefense =
    DefenseModeResolver.resolve(DefenseStrategyMode.AUTO_ADAPTIVE, season.defenseDefaults(), proj);
```

## Documentation

- [Season Import JSON Schema Reference](docs/SEASON_SCHEMA.md)
