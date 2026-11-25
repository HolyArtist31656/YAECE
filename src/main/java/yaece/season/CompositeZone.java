package yaece.season;

import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;
import org.wpilib.math.geometry.Translation2d;

/**
 * Set-theoretic 2D field zone supporting boolean composition ({@code union}, {@code intersect},
 * {@code subtract}, {@code complement}) over polygon regions without external geometry libraries.
 */
public final class CompositeZone {
  private final Predicate<Translation2d> membership;

  private CompositeZone(Predicate<Translation2d> membership) {
    this.membership = Objects.requireNonNull(membership, "membership must not be null");
  }

  /** Creates a leaf zone backed by a 2D polygon in field coordinates. */
  public static CompositeZone ofPolygon(List<Translation2d> polygon) {
    List<Translation2d> copy =
        List.copyOf(Objects.requireNonNull(polygon, "polygon must not be null"));
    return new CompositeZone(pt -> FieldMirroring.polygonContainsPoint(copy, pt));
  }

  /** Creates an axis-aligned rectangular zone {@code [minX, maxX] x [minY, maxY]}. */
  public static CompositeZone ofRect(double minX, double minY, double maxX, double maxY) {
    double x0 = Math.min(minX, maxX);
    double x1 = Math.max(minX, maxX);
    double y0 = Math.min(minY, maxY);
    double y1 = Math.max(minY, maxY);
    return new CompositeZone(
        pt -> pt.getX() >= x0 && pt.getX() <= x1 && pt.getY() >= y0 && pt.getY() <= y1);
  }

  /** Returns true when {@code point} is inside this composite zone. */
  public boolean contains(Translation2d point) {
    Objects.requireNonNull(point, "point must not be null");
    return membership.test(point);
  }

  /** Returns the set union ({@code this OR other}). */
  public CompositeZone union(CompositeZone other) {
    Objects.requireNonNull(other, "other must not be null");
    return new CompositeZone(pt -> this.membership.test(pt) || other.membership.test(pt));
  }

  /** Returns the set intersection ({@code this AND other}). */
  public CompositeZone intersect(CompositeZone other) {
    Objects.requireNonNull(other, "other must not be null");
    return new CompositeZone(pt -> this.membership.test(pt) && other.membership.test(pt));
  }

  /** Returns the set difference ({@code this AND NOT other}). */
  public CompositeZone subtract(CompositeZone other) {
    Objects.requireNonNull(other, "other must not be null");
    return new CompositeZone(pt -> this.membership.test(pt) && !other.membership.test(pt));
  }

  /** Returns the set complement ({@code NOT this}). */
  public CompositeZone complement() {
    return new CompositeZone(pt -> !this.membership.test(pt));
  }
}
