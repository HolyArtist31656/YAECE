# Dynamic Post-Bump Shot Readiness Gate

`ShotReadinessGate.evaluateDynamic(...)` combines:
- **Distance-scaled aim tolerance** (`distanceScaledHeadingToleranceDeg(...)`),
- **Post-bump chassis settle hold** (`recordChassisAccelerationG(accelDeltaG, nowSec)` + `isInPostBumpHold(nowSec)`),
- **Autonomous max-hold fail-safe timeout** to prevent autonomous deadlock, and
- **Volley hysteresis & pose-free SetShot fallback**.
