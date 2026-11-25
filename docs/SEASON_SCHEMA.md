# YAECE Season Import JSON Schema (v1)

YAECE is completely game-agnostic. Each FRC season is described by one UTF-8 JSON file loaded via `SeasonConfig.fromFile(Path)` or `SeasonConfig.fromClasspathResource(String)`. All tactical blocks (`pinRules`, `restrictedZones`, `scoringObjectives`, `defenseDefaults`, `fieldType`, `compositeZones`, `ballisticTables`) are optional and default to modern FRC standards.

## Top-Level Fields

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `schemaVersion` | `int` | Yes | Must be `1`. |
| `seasonName` | `string` | Yes | Season identifier (e.g., `"FRC_2027"`). |
| `fieldType` | `string` | No | Optional (`v2027.5.0-alpha1`): `"WELDED"` or `"ANDYMARK"` perimeter variant (`yaece.season.FieldType`). |
| `fieldLengthMeters` / `fieldWidthMeters` | `double` | Yes | Full field dimensions in meters (`> 0`). |
| `mirrorMode` | `string` | Yes | `"ROTATIONAL_180"`, `"REFLECTION_X"`, or `"NONE"`. |
| `maxTotalHeldGamePieces` | `int` | Yes | Max total game pieces held simultaneously (`>= 1`). |
| `matchDurationSeconds` | `double` | No | Total match duration in seconds (default `150.0`; `160.0` for 2026 REBUILT). |
| `gamePieces` | `array` | Yes | Non-empty game-piece 3D specs & per-piece capacity limits. |
| `scoringSites` | `array` | No | Canonical Blue scoring sites with `autoPoints`/`teleopPoints` (or legacy `points`), `tags`, `stagingHoldPose2d`, and optional SOTM fields (`shootOnTheMoveCapable`, `maxScoringRangeMeters`). |
| `staticObstacles` | `array` | No | 2D polygonal field structures (`blocksVision`, `blocksChassis`) consumed by `FieldPathfinder`. |
| `pinRules` | `object` | No | FRC pinning thresholds (`maxPinSeconds: 3.0`, `backoffTriggerSeconds: 2.2`, `separationDistanceMeters: 1.83`, `cooldownHoldSeconds: 3.2`). |
| `restrictedZones` | `array` | No | `HARD_NO_ENTRY`, `NO_OPPONENT_CONTACT`, or `SOFT_AVOIDANCE` polygons with optional `activeInAuto`, `activeInTeleop`, and `activeWhenMatchTimeRemainingSecLessOrEqual`. |
| `compositeZones` | `array` | No | Optional (`v2027.5.0-alpha1`): Set-theoretic `CompositeZone` definitions (`UNION`, `INTERSECTION`, `DIFFERENCE`, `COMPLEMENT`). |
| `ballisticTables` | `object` | No | Optional (`v2027.5.0-alpha1`): Distance-indexed `BallisticTable` (`BallisticShotParams`: flywheel RPS, hood angle, TOF, heading tolerance) per scoring site. |
| `scoringObjectives` | `array` | No | Linked `BONUS_MATCH_POINTS` or `RANKING_POINT` thresholds (`TOTAL_COUNT`, `PER_SITE_MINIMUM`, or `SAME_SITE_STREAK`). |
| `defenseDefaults` | `object` | No | `defaultPreferredMode` (`HIGHEST_THREAT` or `ZONE_DEFENSE`), lead/trail score margins for `AUTO_ADAPTIVE`, and named `defenseZones`. |
