package yaece.world;

import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;

/** Pure-Java shortest-path quaternion SLERP utility for 6-DoF orientation smoothing. */
public final class QuaternionSlerp {
  private QuaternionSlerp() {}

  /**
   * Computes shortest-path spherical linear interpolation between two 3D orientations.
   *
   * @param a starting orientation
   * @param b target orientation
   * @param alpha blend factor in [0, 1]
   * @return interpolated {@link Rotation3d}
   */
  public static Rotation3d slerp(Rotation3d a, Rotation3d b, double alpha) {
    double t = Math.max(0.0, Math.min(1.0, alpha));
    if (t <= 0.0) {
      return a;
    }
    if (t >= 1.0) {
      return b;
    }

    Quaternion qa = a.getQuaternion();
    Quaternion qb = b.getQuaternion();
    double aw = qa.getW();
    double ax = qa.getX();
    double ay = qa.getY();
    double az = qa.getZ();

    double bw = qb.getW();
    double bx = qb.getX();
    double by = qb.getY();
    double bz = qb.getZ();

    double dot = aw * bw + ax * bx + ay * by + az * bz;
    if (dot < 0.0) {
      dot = -dot;
      bw = -bw;
      bx = -bx;
      by = -by;
      bz = -bz;
    }

    double w, x, y, z;
    if (dot > 0.9995) {
      w = aw + t * (bw - aw);
      x = ax + t * (bx - ax);
      y = ay + t * (by - ay);
      z = az + t * (bz - az);
    } else {
      double theta0 = Math.acos(Math.min(1.0, dot));
      double sinTheta0 = Math.sin(theta0);
      double s0 = Math.sin((1.0 - t) * theta0) / sinTheta0;
      double s1 = Math.sin(t * theta0) / sinTheta0;
      w = s0 * aw + s1 * bw;
      x = s0 * ax + s1 * bx;
      y = s0 * ay + s1 * by;
      z = s0 * az + s1 * bz;
    }

    double norm = Math.sqrt(w * w + x * x + y * y + z * z);
    if (norm < 1e-12) {
      return a;
    }
    return new Rotation3d(new Quaternion(w / norm, x / norm, y / norm, z / norm));
  }

  /** Returns the geodesic angular distance in radians between two 3D orientations. */
  public static double angularDistanceRad(Rotation3d a, Rotation3d b) {
    Quaternion qa = a.getQuaternion();
    Quaternion qb = b.getQuaternion();
    double dot =
        Math.abs(
            qa.getW() * qb.getW()
                + qa.getX() * qb.getX()
                + qa.getY() * qb.getY()
                + qa.getZ() * qb.getZ());
    return 2.0 * Math.acos(Math.min(1.0, dot));
  }
}
