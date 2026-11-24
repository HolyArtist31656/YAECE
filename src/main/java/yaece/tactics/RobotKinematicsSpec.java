package yaece.tactics;

/**
 * Simplified chassis kinematics and mechanism cycle-time parameters used for travel-time races and
 * points-per-second (PPS) calculations.
 */
public record RobotKinematicsSpec(
    double maxSpeedMps,
    double maxAccelMps2,
    double bumperRadiusMeters,
    double nominalIntakeCycleSec,
    double nominalScoreCycleSec,
    boolean stowMechanismsInDefense) {

  /** Sensible default for a modern FRC swerve chassis (4.5 m/s, 6.0 m/s^2, 0.45 m bumper). */
  public static final RobotKinematicsSpec FRC_SWERVE_DEFAULT =
      new RobotKinematicsSpec(4.5, 6.0, 0.45, 0.65, 0.85, true);

  public RobotKinematicsSpec {
    if (!Double.isFinite(maxSpeedMps) || maxSpeedMps <= 0.0) {
      throw new IllegalArgumentException("maxSpeedMps must be > 0");
    }
    if (!Double.isFinite(maxAccelMps2) || maxAccelMps2 <= 0.0) {
      throw new IllegalArgumentException("maxAccelMps2 must be > 0");
    }
    if (!Double.isFinite(bumperRadiusMeters) || bumperRadiusMeters <= 0.0) {
      throw new IllegalArgumentException("bumperRadiusMeters must be > 0");
    }
    if (!Double.isFinite(nominalIntakeCycleSec) || nominalIntakeCycleSec < 0.0) {
      throw new IllegalArgumentException("nominalIntakeCycleSec must be >= 0");
    }
    if (!Double.isFinite(nominalScoreCycleSec) || nominalScoreCycleSec < 0.0) {
      throw new IllegalArgumentException("nominalScoreCycleSec must be >= 0");
    }
  }

  /**
   * Computes minimum rest-to-rest trapezoidal travel time in seconds for {@code distanceMeters}.
   */
  public double estimateTravelTimeSec(double distanceMeters) {
    double d = Math.max(0.0, distanceMeters);
    if (d < 1e-4) {
      return 0.0;
    }
    double accelDist = (maxSpeedMps * maxSpeedMps) / maxAccelMps2;
    if (d <= accelDist) {
      return 2.0 * Math.sqrt(d / maxAccelMps2);
    }
    return (2.0 * maxSpeedMps / maxAccelMps2) + ((d - accelDist) / maxSpeedMps);
  }
}
