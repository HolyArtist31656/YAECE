# YAECE — Yet Another Encounter Control Engine

**Yet Another Encounter Control Engine (YAECE)** is a game-agnostic, strictly **sensor-agnostic**, zero-dependency FRC Java 25 vendordep for **WPILib 2027 (`2027.0.0-alpha-7`)** and **Commands v3**. No specific camera, LiDAR, or vision library is required—teams feed standardized `Pose3d` / 6-DoF observations from any source, while optional bridges integrate AdvantageScope, QuestNav, PhotonVision, LiDAR, and YAMS/YAGSL.

## Locked 2027 Stack

| Property | Value |
| --- | --- |
| WPILib Version | `2027.0.0-alpha-7` (`2027_alpha7`, `org.wpilib.*`) |
| Java Toolchain | Java 25 (`C:\Users\Public\wpilib\2027_alpha7\jdk`) |
| Gradle | `9.4.1` |
| Command Framework | Commands v3 only (`org.wpilib.command3`) |
| Third-Party Runtime JARs | **0** (internal `MinimalJson` parser + WPILib/WPIMath only) |
| Formatting | Spotless `8.0.0` (`googleJavaFormat('1.28.0')`) |
| Target Hardware | 2 GB RAM Limelight SystemCore (CM5 Cortex-A76) |

## Included Packages (`v2027.1.0-alpha1`, 33/33 JUnit 5 Tests)

- `yaece.json`: Zero-dependency RFC 8259 parser (`MinimalJson`).
- `yaece.season`: Game-agnostic `SeasonConfig`, `FieldMirroring`, `ScoringIntervalSpec`, pin rules, restricted zones, RP objectives, defense corridors, plus bundled `season_2026_rebuilt.json`.
- `yaece.ledger`: Timestamp-reconciled `MatchScoreLedger`, EWLS `ScoreProjection`, and `DefenseModeResolver`.
- `yaece.perception` & `yaece.world`: 100 Hz ego pose ring buffer, 6-DoF piece SLERP/EKF, 5-robot 2-mode IMM tracker, `HungarianMatcher`, `LidarCameraRobotFuser`, and `PinGuardTracker`.
- `yaece.console`: Zero-wiring NT4 console under `yaece/console/*`.
- `yaece.tactics` & `yaece.commands3`: `YaeceEngine`, `EncounterArbiter` (PPS/RP utility, dynamic site gating/multipliers, closing-window guard, full-hopper/pre-open staging), `DefenseInterceptor`, and `YaeceCommandFactory.runAutonomousEncounter`.
- `yaece.bridges`: Optional `AdvantageScopeTelemetryBridge`, `Coprocessor6DoFNtBridge`, `QuestNavAdapter`, `PhotonVisionAdapter`, and `YamsSwerveHooks`.

## Build & Local Install

    $env:JAVA_HOME = "C:\Users\Public\wpilib\2027_alpha7\jdk"
    $env:Path      = "$env:JAVA_HOME\bin;$env:Path"
    .\gradlew.bat spotlessApply test publishToMavenLocal