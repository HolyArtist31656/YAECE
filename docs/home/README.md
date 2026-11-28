# Yet Another Encounter Control Engine (YAECE)

**YAECE** is a zero-dependency FRC 2027 (WPILib `2027.0.0-alpha-8`, Java 25, Commands v3) tactical encounter engine, 6-DoF field world model, scouting-driven defense planner, and dynamic shoot-on-the-move (SOTM) readiness gate.

## Ecosystem Boundaries
- **YAMS / YAGSL**: Mechanism & swerve motor control.
- **YALL**: Camera wrappers, AprilTag localization, and multi-camera pose fusion (`YAECE` ingests YALL detections via vendor-neutral `YallTargetBridge`).
- **YAPL**: Field pathfinding, polyline following, and polar close-in alignment.
- **YACS**: 3D CAD physics, chassis dynamics, and ballistics simulation.
- **YAECE**: Tactical encounter arbitration, 6-DoF object/robot tracking, scouting priors, and shot gating.
