// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.world;

import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Thread-safe, fixed-capacity ring buffer of timestamped ego {@link Pose3d} samples supporting
 * latency-compensated pose lookup, windowed peak angular-rate queries (FRC 254), and pitch/roll
 * bump-traversal odometry attenuation (FRC 6328 2026).
 */
public final class EgoPoseBuffer {
  private static final double DEFAULT_FULL_TILT_ATTENUATION_DEG = 25.0;

  private final int capacity;
  private final double[] timestamps;
  private final Pose3d[] poses;
  private int head = 0;
  private int size = 0;

  /**
   * Creates a ring buffer holding up to {@code capacity} ego samples (e.g., 200 = 2 s @ 100 Hz).
   */
  public EgoPoseBuffer(int capacity) {
    if (capacity < 2) {
      throw new IllegalArgumentException("EgoPoseBuffer capacity must be >= 2");
    }
    this.capacity = capacity;
    this.timestamps = new double[capacity];
    this.poses = new Pose3d[capacity];
  }

  /** Adds a monotonic timestamped ego pose sample. */
  public synchronized void addSample(Pose3d pose3d, double timestampSec) {
    Objects.requireNonNull(pose3d, "pose3d must not be null");
    if (!Double.isFinite(timestampSec) || timestampSec < 0.0) {
      throw new IllegalArgumentException("timestampSec must be finite and >= 0");
    }
    if (size > 0) {
      int lastIdx = (head - 1 + capacity) % capacity;
      if (timestampSec < timestamps[lastIdx]) {
        return;
      }
      if (Math.abs(timestampSec - timestamps[lastIdx]) < 1e-6) {
        poses[lastIdx] = pose3d;
        return;
      }
    }
    timestamps[head] = timestampSec;
    poses[head] = pose3d;
    head = (head + 1) % capacity;
    if (size < capacity) {
      size++;
    }
  }

  /** Returns the most recent ego pose, or empty if no samples have been recorded. */
  public synchronized Optional<Pose3d> latestPose() {
    if (size == 0) {
      return Optional.empty();
    }
    int lastIdx = (head - 1 + capacity) % capacity;
    return Optional.of(poses[lastIdx]);
  }

  /**
   * Returns the latency-matched ego {@link Pose3d} at {@code queryTimestampSec}, interpolating
   * between surrounding samples or clamping to the nearest endpoint.
   */
  public synchronized Optional<Pose3d> sampleAt(double queryTimestampSec) {
    if (size == 0) {
      return Optional.empty();
    }
    int oldestIdx = (head - size + capacity) % capacity;
    int newestIdx = (head - 1 + capacity) % capacity;
    if (size == 1 || queryTimestampSec <= timestamps[oldestIdx]) {
      return Optional.of(poses[oldestIdx]);
    }
    if (queryTimestampSec >= timestamps[newestIdx]) {
      return Optional.of(poses[newestIdx]);
    }

    for (int offset = 1; offset < size; offset++) {
      int hi = (newestIdx - offset + 1 + capacity) % capacity;
      int lo = (newestIdx - offset + capacity) % capacity;
      double tLo = timestamps[lo];
      double tHi = timestamps[hi];
      if (queryTimestampSec >= tLo && queryTimestampSec <= tHi) {
        double span = tHi - tLo;
        double alpha = span > 1e-9 ? (queryTimestampSec - tLo) / span : 0.0;
        Pose3d pLo = poses[lo];
        Pose3d pHi = poses[hi];
        Translation3d trans =
            new Translation3d(
                pLo.getX() + alpha * (pHi.getX() - pLo.getX()),
                pLo.getY() + alpha * (pHi.getY() - pLo.getY()),
                pLo.getZ() + alpha * (pHi.getZ() - pLo.getZ()));
        Rotation3d rot = QuaternionSlerp.slerp(pLo.getRotation(), pHi.getRotation(), alpha);
        return Optional.of(new Pose3d(trans, rot));
      }
    }
    return Optional.of(poses[newestIdx]);
  }

  /** Estimates planar ego velocity {@code [vx, vy]} (m/s) over the most recent samples. */
  public synchronized double[] estimatePlanarVelocity() {
    if (size < 2) {
      return new double[] {0.0, 0.0};
    }
    int newestIdx = (head - 1 + capacity) % capacity;
    int prevIdx = (newestIdx - 1 + capacity) % capacity;
    double dt = timestamps[newestIdx] - timestamps[prevIdx];
    if (dt < 1e-4) {
      return new double[] {0.0, 0.0};
    }
    Pose3d pNew = poses[newestIdx];
    Pose3d pOld = poses[prevIdx];
    return new double[] {(pNew.getX() - pOld.getX()) / dt, (pNew.getY() - pOld.getY()) / dt};
  }

  /**
   * Returns the peak absolute yaw angular velocity (rad/s) observed across consecutive samples
   * overlapping {@code [startSec, endSec]} (inspired by FRC 254's {@code ±0.3 s} vision rate gate).
   */
  public synchronized double maxAbsYawRateInWindow(double startSec, double endSec) {
    if (size < 2 || endSec < startSec) {
      return 0.0;
    }
    int oldestIdx = (head - size + capacity) % capacity;
    double maxRate = 0.0;
    for (int i = 1; i < size; i++) {
      int lo = (oldestIdx + i - 1) % capacity;
      int hi = (oldestIdx + i) % capacity;
      double tLo = timestamps[lo];
      double tHi = timestamps[hi];
      if (tHi < startSec || tLo > endSec) {
        continue;
      }
      double dt = tHi - tLo;
      if (dt < 1e-4) {
        continue;
      }
      double dYaw = wrapRad(poses[hi].getRotation().getZ() - poses[lo].getRotation().getZ());
      maxRate = Math.max(maxRate, Math.abs(dYaw) / dt);
    }
    return maxRate;
  }

  /**
   * Computes tilt-compensated planar odometry attenuation {@code [0..1]} for bump/ramp traversal
   * (matching FRC 6328's 2026 REBUILT formula: {@code 1 - clamp(tiltDeg / 25.0, 0, 1)} where {@code
   * tiltDeg = deg(acos(cos(pitch) * cos(roll)))}).
   */
  public static double tiltOdometryScale(double pitchRad, double rollRad) {
    double cosTilt = Math.max(-1.0, Math.min(1.0, Math.cos(pitchRad) * Math.cos(rollRad)));
    double tiltDeg = Math.toDegrees(Math.acos(cosTilt));
    double ratio = Math.max(0.0, Math.min(1.0, tiltDeg / DEFAULT_FULL_TILT_ATTENUATION_DEG));
    return 1.0 - ratio;
  }

  public synchronized void clear() {
    head = 0;
    size = 0;
  }

  private static double wrapRad(double rad) {
    double r = (rad + Math.PI) % (2.0 * Math.PI);
    if (r < 0.0) {
      r += 2.0 * Math.PI;
    }
    return r - Math.PI;
  }
}
