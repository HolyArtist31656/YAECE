# Installation & Season Configs

Install `vendordeps/yaece_commands3.json` in your WPILib 2027 project, or publish locally:

```bash
./gradlew publishToMavenLocal
```

Load a season rulebook JSON (scoring zones, hub shift windows, endgame no-contact rules, and game-piece specs) with `SeasonConfig.fromClasspathResource(...)`.
