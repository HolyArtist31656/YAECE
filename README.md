# YAECE — Yet Another Encounter Control Engine

**Yet Another Encounter Control Engine (YAECE)** is a game-agnostic, strictly **sensor-agnostic**, zero-dependency FRC Java 25 vendordep for **WPILib 2027 (`2027.0.0-alpha-7`)** and **Commands v3**. No specific camera, LiDAR, or vision library is required—teams feed standardized `Pose3d` / 6-DoF observations from any source, while optional zero-dependency bridges integrate QuestNav, PhotonVision, LiDAR, AdvantageScope NT4, WPILib DataLog, YAMS/YAGSL, and YACSim 3D.

## Locked 2027 Stack (Mirrors YACSim)

| Property | Value |
| --- | --- |
| Current Release | **`v2027.5.0-alpha1`** (87 Java classes, **54/54 JUnit 5 tests**) |
| WPILib Version | `2027.0.0-alpha-7` (`2027_alpha7`, `org.wpilib.*`) |
| Java Toolchain | Java 25 (`C:\Users\Public\wpilib\2027_alpha7\jdk`) |
| Gradle | `9.4.1` |
| Command Framework | Commands v3 only (`org.wpilib.command3`) |
| Third-Party Runtime JARs | **0** (internal `MinimalJson` parser + WPILib/WPIMath only) |
| Formatting | Spotless (`googleJavaFormat`) |
| Target Hardware | 2 GB RAM Limelight SystemCore (`-Xmx256m` verified) |

## Capabilities (`v2027.5.0-alpha1` — Phases 1–8 + Section 6 Core Closure)

1. **Vendordep & GitHub Pages Distribution**: `build.gradle`, `gradle.properties`, `vendordeps/yaece_commands3.json`, and `.github/workflows/publish-pages.yml` (tag-gated `v*` releases).
2. **Zero-Dependency JSON Engine (`yaece.json`)**: Strict RFC 8259 `MinimalJson` parser with line/column diagnostics.
3. **Extended Season Configuration (`yaece.season`)**: `SeasonConfig` with `FieldType` (`WELDED` / `ANDYMARK`), `CompositeZone` boolean operations (`UNION`, `INTERSECTION`, `DIFFERENCE`, `COMPLEMENT`), `autoPoints`/`teleopPoints`, site `tags`, `pinRules` (FRC 3 s / 1.83 m), `restrictedZones` (`HARD_NO_ENTRY`, `NO_OPPONENT_CONTACT`, `SOFT_AVOIDANCE` with Endgame time windows), `scoringObjectives`, `defenseDefaults`, and pre-cached `FieldMirroring`.
4. **6-DoF World Model & Multi-Agent Tracking (`yaece.world`)**: Thread-safe `FieldWorldModel`, 5-robot IMM tracker, `EgoPoseBuffer` with bump tilt odometry attenuation, `VisionPoseGate`, `PieceDetectionFilter` (4-gate intake filter), `CollisionDetector`, and `RobotHealthState` (odometry confidence, skid/tilt, brownout throttle, and impact signals).
5. **Tactical Arbitration, Pathfinding & Elite Scoring (`yaece.tactics`, `yaece.ballistics`)**: Epoch-cached A\* `FieldPathfinder` (3 clearance profiles + exact AABB inflation), velocity-projected arrival-race arbiter, `PolarAutoAlignController`, `ShootOnTheMoveSolver`, JSON-loadable `BallisticTable` (`BallisticShotParams`), `ShotReadinessGate` (volley hysteresis, timeout, bypass, and pose-free SetShot fallback), `HazardAutoTuck`, and `YaeceDiagnostic` boot validation.
6. **Live Score Ledger & Scouting Priors (`yaece.ledger`, `yaece.scouting`)**: `MatchScoreLedger` synced with `DynamicScoringState` multipliers, `ScoreProjection`, `OptimizationObjective` (`MATCH_POINTS` vs. `RANKING_POINTS`), `DefenseModeResolver`, and `ScoutingPriorRegistry` (`OpponentProfile`, `RobotRolePrior`).
7. **Deterministic Replay & Bridges (`yaece.replay`, `yaece.bridges`)**: Tick-by-tick `YaeceInputRecorder` + `YaeceReplayHarness` (JSONL), `YaeceDataLogSink` (direct WPILib `.wpilog` recording), `AdvantageScopeTelemetryBridge`, `CoprocessorNtBridge`, and `YacSim3dBridge` (live-verified with YACSim `v2027.5.0-alpha1` in `YAECE-RebuiltSim-Example`).

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

## Building & Testing Locally

```powershell
.\gradlew spotlessApply test publishToMavenLocal "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## Documentation

- [YAECE User & Implementation Guide (Live Site)](https://holyartist31656.github.io/YAECE/)
- [Hosted Javadoc](https://holyartist31656.github.io/YAECE/javadoc/index.html)
- **Online Vendordep JSON**: `https://holyartist31656.github.io/YAECE/vendordeps/yaece_commands3.json`
- [Season Import JSON Schema Reference](docs/SEASON_SCHEMA.md)
- [Coprocessor NT4 Wire Protocol](docs/COPROCESSOR_WIRE_PROTOCOL.md)
