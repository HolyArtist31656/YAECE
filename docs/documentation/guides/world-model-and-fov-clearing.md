# 6-DoF World Model & FOV Negative Clearing

`FieldWorldModel` fuses 6-DoF game-piece detections (Hungarian GNN + EKF + quaternion SLERP) and up to 5 moving robots (2-mode IMM stationary/constant-velocity filter) at 50–100 Hz.

## Occlusion-Aware Camera FOV Clearing
Register a camera frustum with `FieldWorldModel.CameraFovSpec` and call `recordCameraFrame(camera, detections, timestampSec)` or `recordEmptyCameraObservation(camera, timestampSec)`:
- Pieces inside the unoccluded frustum missing for 2 consecutive frames clear immediately (e.g., stolen by an opponent).
- Pieces outside the FOV or occluded behind another tracked robot remain preserved.
