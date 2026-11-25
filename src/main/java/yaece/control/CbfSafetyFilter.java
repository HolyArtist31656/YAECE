package yaece.control;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;

/**
 * Zero-solver closed-form Control Barrier Function (CBF-QP) velocity safety filter for holonomic
 * swerve drivetrains, eliminating artificial-potential-field oscillation in tight doorways while
 * differentiating Nash (50/50 alliance) vs. Stackelberg (100% ego vs. opponent) avoidance
 * responsibility.
 *
 * <p>For each obstacle {@code i} with safety clearance radius {@code R_i}, uses the signed-distance
 * barrier {@code h(x) = ||p - p_i|| - R_i} with unit normal {@code n = (p - p_i) / ||p - p_i||}.
 * The forward-invariance condition is {@code n . (u - v_i) + alpha * h >= 0}. When violated ({@code
 * Psi < 0}), the minimum-norm closed-form projection applies {@code u* = u_des - n * Psi *
 * responsibility}.
 */
public final class CbfSafetyFilter {

  /** Agent cooperation role determining Nash vs. Stackelberg avoidance responsibility. */
  public enum AgentRole {
    /** Static field obstacle: ego takes 100% avoidance responsibility. */
    STATIC_OBSTACLE(1.0),
    /** Alliance partner: cooperative Nash 50/50 split responsibility. */
    ALLIANCE_PARTNER(0.5),
    /** Opponent robot: conservative Stackelberg 100% ego responsibility. */
    OPPONENT(1.0);

    private final double defaultResponsibility;

    AgentRole(double defaultResponsibility) {
      this.defaultResponsibility = defaultResponsibility;
    }

    /**
     * @return Default avoidance responsibility weight in {@code (0, 1]}.
     */
    public double defaultResponsibility() {
      return defaultResponsibility;
    }
  }

  /**
   * Circular obstacle or dynamic robot agent for CBF barrier evaluation.
   *
   * @param id Unique obstacle/agent identifier.
   * @param positionMeters Field-frame X/Y center in meters.
   * @param velocityMps Field-frame X/Y velocity in m/s (zero for static obstacles).
   * @param combinedRadiusMeters Sum of ego bumper radius + obstacle radius + safety margin in
   *     meters.
   * @param role Cooperation role (static, ally Nash, or opponent Stackelberg).
   */
  public record BarrierObstacle(
      String id,
      Translation2d positionMeters,
      Translation2d velocityMps,
      double combinedRadiusMeters,
      AgentRole role) {
    public BarrierObstacle {
      Objects.requireNonNull(id, "id");
      Objects.requireNonNull(positionMeters, "positionMeters");
      Objects.requireNonNull(velocityMps, "velocityMps");
      Objects.requireNonNull(role, "role");
      if (combinedRadiusMeters <= 0.0 || !Double.isFinite(combinedRadiusMeters)) {
        throw new IllegalArgumentException("combinedRadiusMeters must be positive and finite.");
      }
    }
  }

  /**
   * Filtered velocity command and diagnostic barrier status.
   *
   * @param safeVelocityMps Filtered field-frame X/Y velocity in m/s.
   * @param filterActive {@code true} if at least one barrier constraint modified {@code
   *     desiredVelocityMps}.
   * @param minBarrierValue Smallest signed-distance barrier value {@code h} in meters across all
   *     obstacles (negative means inside safety margin).
   * @param activeConstraintCount Number of obstacles that actively constrained velocity.
   */
  public record FilterResult(
      Translation2d safeVelocityMps,
      boolean filterActive,
      double minBarrierValue,
      int activeConstraintCount) {}

  private final double alpha;
  private final double maxSpeedMps;

  /**
   * Creates a closed-form CBF safety filter.
   *
   * @param alpha Class-K barrier decay gain in {@code [0.5, 1.0]} (recommended for
   *     double-integrator swerve).
   * @param maxSpeedMps Maximum allowed linear speed magnitude in m/s.
   */
  public CbfSafetyFilter(double alpha, double maxSpeedMps) {
    if (alpha <= 0.0 || !Double.isFinite(alpha)) {
      throw new IllegalArgumentException("alpha must be positive and finite.");
    }
    if (maxSpeedMps <= 0.0 || !Double.isFinite(maxSpeedMps)) {
      throw new IllegalArgumentException("maxSpeedMps must be positive and finite.");
    }
    this.alpha = alpha;
    this.maxSpeedMps = maxSpeedMps;
  }

  /**
   * @return Configured CBF decay gain {@code alpha}.
   */
  public double getAlpha() {
    return alpha;
  }

  /**
   * Filters {@code desiredVelocityMps} against all provided obstacles using iterative closed-form
   * half-space projection (zero external QP solver, zero heap allocation in inner loop).
   *
   * @param egoPositionMeters Current robot X/Y position in meters.
   * @param desiredVelocityMps Nominal desired field-frame X/Y velocity in m/s.
   * @param obstacles Active static and dynamic barrier obstacles.
   * @return Safe filtered velocity and diagnostics.
   */
  public FilterResult filter(
      Translation2d egoPositionMeters,
      Translation2d desiredVelocityMps,
      List<BarrierObstacle> obstacles) {
    Objects.requireNonNull(egoPositionMeters, "egoPositionMeters");
    Objects.requireNonNull(desiredVelocityMps, "desiredVelocityMps");
    Objects.requireNonNull(obstacles, "obstacles");

    double ux = desiredVelocityMps.getX();
    double uy = desiredVelocityMps.getY();
    double minH = Double.POSITIVE_INFINITY;
    boolean anyActive = false;
    int activeCount = 0;

    // 3 projection passes handle multi-obstacle corridors (e.g., 0.7 m doorways) cleanly
    for (int pass = 0; pass < 3; pass++) {
      boolean modifiedThisPass = false;
      int passActiveCount = 0;
      for (int i = 0; i < obstacles.size(); i++) {
        BarrierObstacle obs = obstacles.get(i);
        double dx = egoPositionMeters.getX() - obs.positionMeters().getX();
        double dy = egoPositionMeters.getY() - obs.positionMeters().getY();
        double dist = Math.hypot(dx, dy);
        double h = dist - obs.combinedRadiusMeters();
        if (pass == 0 && h < minH) {
          minH = h;
        }

        // Unit normal of signed-distance barrier h(x) = ||p - p_obs|| - R
        double nx;
        double ny;
        if (dist > 1e-6) {
          nx = dx / dist;
          ny = dy / dist;
        } else {
          nx = 1.0;
          ny = 0.0;
        }

        double relVx = ux - obs.velocityMps().getX();
        double relVy = uy - obs.velocityMps().getY();
        double psi = nx * relVx + ny * relVy + alpha * h;

        if (psi < 0.0) {
          double weight = obs.role().defaultResponsibility();
          ux -= nx * psi * weight;
          uy -= ny * psi * weight;
          anyActive = true;
          modifiedThisPass = true;
          passActiveCount++;
        }
      }
      if (pass == 0) {
        activeCount = passActiveCount;
      }
      if (!modifiedThisPass) {
        break;
      }
    }

    // Clamp final linear speed to maxSpeedMps
    double speed = Math.hypot(ux, uy);
    if (speed > maxSpeedMps) {
      double s = maxSpeedMps / speed;
      ux *= s;
      uy *= s;
    }

    return new FilterResult(
        new Translation2d(ux, uy),
        anyActive,
        obstacles.isEmpty() ? Double.POSITIVE_INFINITY : minH,
        activeCount);
  }
}
