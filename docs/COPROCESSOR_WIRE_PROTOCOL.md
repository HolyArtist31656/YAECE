# YAECE Coprocessor NT4 Wire Protocol

Any external coprocessor (Orange Pi 5, Rubik Pi 3, Jetson, OAK-D, RealSense, or USB 3.0 2D LiDAR node) can stream 6-DoF game pieces, LiDAR bumper clusters, and multi-camera bumper bearings directly into YAECE with **zero custom Java code** by calling `engine.enableCoprocessorNtBridge()`.

## NetworkTables 4 Topics (`yaece/perception/*`)

| Topic | Type | Stride | Layout per Element |
| --- | --- | --- | --- |
| `yaece/perception/pieces6dof` | `double[]` | 11 | `[classId, x, y, z, rollRad, pitchRad, yawRad, posConf, orientConf, frameFlag(0=FIELD,1=ROBOT), timestampSec]` |
| `yaece/perception/lidarClusters` | `double[]` | 5 | `[centerXMeters, centerYMeters, widthMeters, confidence, timestampSec]` |
| `yaece/perception/cameraBearings` | `double[]` | 5 | `[bearingRad, affFlag(0=UNKNOWN,1=ALLY,2=OPPONENT), confidence, estRangeMeters, timestampSec]` |

## Example Python Coprocessor Publisher (`pyntcore`)

```python
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("yaece-vision-node")
inst.setServerTeam(2181)

table = inst.getTable("yaece/perception")
pieces_pub = table.getDoubleArrayTopic("pieces6dof").publish()
lidar_pub = table.getDoubleArrayTopic("lidarClusters").publish()
bearings_pub = table.getDoubleArrayTopic("cameraBearings").publish()

# Publish one 6-DoF game piece in robot frame (frameFlag=1)
pieces_pub.set([0, 1.85, -0.30, 0.06, 0.0, 0.0, 0.785, 0.94, 0.90, 1, 42.15])
# Publish one 2D LiDAR bumper cluster + matching opponent camera bearing
lidar_pub.set([2.40, 0.20, 0.85, 0.92, 42.15])
bearings_pub.set([0.08, 2, 0.95, 2.45, 42.15])
```
