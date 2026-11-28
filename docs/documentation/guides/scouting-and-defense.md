# Scouting Profiles, 3v3 Roster & Defense

- **`OpponentProfile`**: Encodes bumper dimensions, max speed, expected scoring rate, per-site preference multipliers, and defense susceptibility.
- **`MatchRoster`**: Enforces valid 3v3 FRC rosters (`MAX_ALLIES = 2`, `MAX_OPPONENTS = 3`) with zero duplicate or overlapping team numbers and JSON loading (`MatchRoster.fromJson`).
- **`DefenseInterceptor`**: Uses `ScoutingPriorRegistry` to prioritize high-threat scorers while shifting defensive blocking poses clear of charging alliance partner corridors and active endgame no-contact zones.
