package yaece.tactics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicLong;
import org.wpilib.math.geometry.Translation2d;
import yaece.season.AllianceSide;
import yaece.season.FieldMirroring;
import yaece.season.MatchPeriod;
import yaece.season.RestrictedZoneSpec;
import yaece.season.SeasonConfig;
import yaece.season.StaticObstacleSpec;
import yaece.season.ZoneRuleType;

/**
 * Lightweight, zero-dependency 2D visibility-graph A* pathfinder over {@link StaticObstacleSpec}
 * and active {@link ZoneRuleType#HARD_NO_ENTRY} polygons.
 *
 * <p>Incorporates three elite FRC pathfinding safeguards:
 *
 * <ul>
 *   <li>Multi-profile clearance padding ({@link ClearanceProfile#AUTO_TRANSIT}, {@link
 *       ClearanceProfile#TELEOP_TIGHT}, {@link ClearanceProfile#HOLD_STAGE_BACKOFF}),
 *   <li>Monotonic {@code obstacleEpoch}-keyed bounded LRU cache (128 entries) that prevents both
 *       per-tick obstacle-set copy churn and stale dynamic-obstacle cache hits,
 *   <li>Fast line-of-sight check returning direct Euclidean distance in microseconds when clear.
 * </ul>
 */
public final class FieldPathfinder {
  private static final int MAX_CACHE_SIZE = 128;
  private static final double CACHE_QUANTIZE_METERS = 0.20;

  /** Context-specific obstacle padding profiles (inspired by Team 254's multi-navgrid design). */
  public enum ClearanceProfile {
    /** Wider safety margin for high-speed autonomous transit. */
    AUTO_TRANSIT(0.16),
    /** Tighter clearance margin for agile teleop and close-in pickups. */
    TELEOP_TIGHT(0.08),
    /** Extra-conservative backoff clearance for staging and defensive retreats. */
    HOLD_STAGE_BACKOFF(0.24);

    private final double extraPadMeters;

    ClearanceProfile(double extraPadMeters) {
      this.extraPadMeters = extraPadMeters;
    }

    public double extraPadMeters() {
      return extraPadMeters;
    }
  }

  public record PathResult(
      double totalDistanceMeters,
      Translation2d nextWaypoint,
      List<Translation2d> waypoints,
      boolean detourRequired) {}

  private record CacheKey(
      int sx,
      int sy,
      int gx,
      int gy,
      AllianceSide alliance,
      MatchPeriod period,
      ClearanceProfile profile,
      long epoch) {}

  private final SeasonConfig season;
  private final double egoBumperRadiusMeters;
  private final AtomicLong obstacleEpoch = new AtomicLong(0L);
  private final Map<CacheKey, PathResult> pathCache =
      new LinkedHashMap<>(MAX_CACHE_SIZE + 1, 0.75f, true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<CacheKey, PathResult> eldest) {
          return size() > MAX_CACHE_SIZE;
        }
      };

  public FieldPathfinder(SeasonConfig season, double egoBumperRadiusMeters) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.egoBumperRadiusMeters = Math.max(0.1, egoBumperRadiusMeters);
  }

  /** Advances the obstacle epoch, immediately invalidating cached paths from earlier epochs. */
  public long bumpObstacleEpoch() {
    return obstacleEpoch.incrementAndGet();
  }

  public long getObstacleEpoch() {
    return obstacleEpoch.get();
  }

  public synchronized int getCacheSize() {
    return pathCache.size();
  }

  public PathResult findPath(
      Translation2d start,
      Translation2d goal,
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec) {
    ClearanceProfile profile =
        period == MatchPeriod.AUTO ? ClearanceProfile.AUTO_TRANSIT : ClearanceProfile.TELEOP_TIGHT;
    return findPath(start, goal, alliance, period, matchTimeRemainingSec, profile);
  }

  /** Computes an obstacle-free 2D path using the specified {@link ClearanceProfile}. */
  public PathResult findPath(
      Translation2d start,
      Translation2d goal,
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      ClearanceProfile profile) {
    Objects.requireNonNull(start, "start must not be null");
    Objects.requireNonNull(goal, "goal must not be null");
    Objects.requireNonNull(profile, "profile must not be null");

    long epoch = obstacleEpoch.get();
    CacheKey key =
        new CacheKey(
            quantize(start.getX()),
            quantize(start.getY()),
            quantize(goal.getX()),
            quantize(goal.getY()),
            alliance,
            period,
            profile,
            epoch);

    synchronized (pathCache) {
      PathResult cached = pathCache.get(key);
      if (cached != null) {
        return cached;
      }
    }

    PathResult computed =
        computePath(start, goal, alliance, period, matchTimeRemainingSec, profile);
    synchronized (pathCache) {
      pathCache.put(key, computed);
    }
    return computed;
  }

  private PathResult computePath(
      Translation2d start,
      Translation2d goal,
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      ClearanceProfile profile) {
    List<List<Translation2d>> activeInflated =
        collectActiveInflatedPolygons(alliance, period, matchTimeRemainingSec, profile);

    if (activeInflated.isEmpty() || !segmentIntersectsAny(start, goal, activeInflated)) {
      return new PathResult(start.getDistance(goal), goal, List.of(goal), false);
    }

    List<Translation2d> nodes = new ArrayList<>();
    nodes.add(start);
    for (int pIdx = 0; pIdx < activeInflated.size(); pIdx++) {
      List<Translation2d> poly = activeInflated.get(pIdx);
      for (Translation2d corner : poly) {
        if (isInsideField(corner) && !isInsideOtherPolygon(corner, activeInflated, pIdx)) {
          nodes.add(corner);
        }
      }
    }
    int goalIdx = nodes.size();
    nodes.add(goal);

    int n = nodes.size();
    double[] gScore = new double[n];
    double[] fScore = new double[n];
    int[] cameFrom = new int[n];
    boolean[] closed = new boolean[n];
    Arrays.fill(gScore, Double.POSITIVE_INFINITY);
    Arrays.fill(fScore, Double.POSITIVE_INFINITY);
    Arrays.fill(cameFrom, -1);

    gScore[0] = 0.0;
    fScore[0] = start.getDistance(goal);

    for (int iter = 0; iter < n; iter++) {
      int current = -1;
      double bestF = Double.POSITIVE_INFINITY;
      for (int i = 0; i < n; i++) {
        if (!closed[i] && fScore[i] < bestF) {
          bestF = fScore[i];
          current = i;
        }
      }
      if (current < 0 || current == goalIdx) {
        break;
      }
      closed[current] = true;
      Translation2d curPt = nodes.get(current);

      for (int nb = 1; nb < n; nb++) {
        if (closed[nb] || nb == current) {
          continue;
        }
        Translation2d nbPt = nodes.get(nb);
        if (segmentIntersectsAny(curPt, nbPt, activeInflated)) {
          continue;
        }
        double tentativeG = gScore[current] + curPt.getDistance(nbPt);
        if (tentativeG < gScore[nb]) {
          cameFrom[nb] = current;
          gScore[nb] = tentativeG;
          fScore[nb] = tentativeG + nbPt.getDistance(goal);
        }
      }
    }

    if (cameFrom[goalIdx] < 0) {
      return new PathResult(start.getDistance(goal) * 1.35, goal, List.of(goal), false);
    }

    List<Translation2d> reversed = new ArrayList<>();
    int trace = goalIdx;
    while (trace > 0 && trace != -1) {
      reversed.add(nodes.get(trace));
      trace = cameFrom[trace];
    }
    Collections.reverse(reversed);
    Translation2d firstWp = reversed.isEmpty() ? goal : reversed.get(0);
    return new PathResult(gScore[goalIdx], firstWp, List.copyOf(reversed), reversed.size() > 1);
  }

  private static int quantize(double meters) {
    return (int) Math.round(meters / CACHE_QUANTIZE_METERS);
  }

  private List<List<Translation2d>> collectActiveInflatedPolygons(
      AllianceSide alliance,
      MatchPeriod period,
      double matchTimeRemainingSec,
      ClearanceProfile profile) {
    List<List<Translation2d>> polys = new ArrayList<>();
    double pad = profile.extraPadMeters();
    for (StaticObstacleSpec obs : season.staticObstacles()) {
      if (!obs.blocksChassis()) {
        continue;
      }
      polys.add(
          inflatePolygon(
              season.resolvedObstaclePolygon(obs, alliance), egoBumperRadiusMeters + pad));
    }
    for (RestrictedZoneSpec zone : season.restrictedZones()) {
      if (zone.ruleType() == ZoneRuleType.HARD_NO_ENTRY
          && zone.isActive(period, matchTimeRemainingSec)) {
        polys.add(
            inflatePolygon(
                season.resolvedZonePolygon(zone, alliance),
                egoBumperRadiusMeters + zone.safetyBufferMeters() + pad));
      }
    }
    return polys;
  }

  private List<Translation2d> inflatePolygon(List<Translation2d> poly, double padMeters) {
    if (poly.isEmpty()) {
      return poly;
    }
    double cx = 0.0;
    double cy = 0.0;
    for (Translation2d p : poly) {
      cx += p.getX();
      cy += p.getY();
    }
    cx /= poly.size();
    cy /= poly.size();

    List<Translation2d> inflated = new ArrayList<>(poly.size());
    for (Translation2d p : poly) {
      double dx = p.getX() - cx;
      double dy = p.getY() - cy;
      double len = Math.hypot(dx, dy);
      double scale = len > 1e-6 ? (len + padMeters * 1.4142135623730951) / len : 1.0;
      double nx = Math.max(0.1, Math.min(season.fieldLengthMeters() - 0.1, cx + dx * scale));
      double ny = Math.max(0.1, Math.min(season.fieldWidthMeters() - 0.1, cy + dy * scale));
      inflated.add(new Translation2d(nx, ny));
    }
    return inflated;
  }

  private boolean isInsideField(Translation2d p) {
    return p.getX() >= egoBumperRadiusMeters
        && p.getX() <= season.fieldLengthMeters() - egoBumperRadiusMeters
        && p.getY() >= egoBumperRadiusMeters
        && p.getY() <= season.fieldWidthMeters() - egoBumperRadiusMeters;
  }

  private static boolean isInsideOtherPolygon(
      Translation2d p, List<List<Translation2d>> polys, int ownerIdx) {
    for (int i = 0; i < polys.size(); i++) {
      if (i != ownerIdx && FieldMirroring.polygonContainsPoint(polys.get(i), p)) {
        return true;
      }
    }
    return false;
  }

  private static boolean segmentIntersectsAny(
      Translation2d a, Translation2d b, List<List<Translation2d>> polys) {
    for (List<Translation2d> poly : polys) {
      int n = poly.size();
      for (int i = 0; i < n; i++) {
        Translation2d c = poly.get(i);
        Translation2d d = poly.get((i + 1) % n);
        if (segmentsProperlyIntersect(a, b, c, d)) {
          return true;
        }
      }
    }
    return false;
  }

  private static boolean segmentsProperlyIntersect(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double eps = 1e-4;
    double ax = a.getX() + eps * (b.getX() - a.getX());
    double ay = a.getY() + eps * (b.getY() - a.getY());
    double bx = b.getX() - eps * (b.getX() - a.getX());
    double by = b.getY() - eps * (b.getY() - a.getY());

    double d1 = cross(c.getX(), c.getY(), d.getX(), d.getY(), ax, ay);
    double d2 = cross(c.getX(), c.getY(), d.getX(), d.getY(), bx, by);
    double d3 = cross(ax, ay, bx, by, c.getX(), c.getY());
    double d4 = cross(ax, ay, bx, by, d.getX(), d.getY());

    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
  }

  private static double cross(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  }
}
