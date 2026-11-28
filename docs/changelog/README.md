# Changelog

## v2027.6.6-alpha1 (2026-11-27)
- Added occlusion-aware camera FOV negative-observation clearing (`FieldWorldModel.CameraFovSpec`) and dynamic distance-scaled post-bump `ShotReadinessGate.evaluateDynamic(...)` (79/79 JUnit 5 + fuzz tests passing).
- Added scouting blocking profiles (`OpponentProfile`), 3v3 `MatchRoster`, and ally-corridor keep-out in `DefenseInterceptor`.
- Synced to WPILib `2027.0.0-alpha-8`, Java 25, and removed duplicate camera wrappers in favor of `YALL` (`YallTargetBridge`).
