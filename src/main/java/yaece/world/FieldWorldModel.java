package yaece.world;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.ObservationFrame;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.season.GamePieceSpec;
import yaece.season.SeasonConfig;

/**
 * Sensor-agnostic 3D FRC field world model simultaneously tracking:
 *
 * <ul>
 *   <li>Ego robot {@link Pose3d} (via 100 Hz-ready {@link EgoPoseBuffer}),
 *   <li>Full 6-DoF game pieces (Hungarian GNN association + position EKF + quaternion SLERP),
 *   <li>Up to 5 other robots (2-mode IMM Stationary/Constant-Velocity filter + occlusion decay),
 *   <li>Per-opponent FRC 3-second pin guard timers ({@link PinGuardTracker}).
 * </ul>
 *
 * <p>Publishes an atomic {@code volatile} {@link WorldSnapshot} on every {@link #step(double)}
 * call.
 */
public final class FieldWorldModel {
  private static final double PIECE_GATE_METERS = 0.65;
  private static final double ROBOT_GATE_METERS = 1.25;
  private static final double PIECE_MAX_AGE_SEC = 1.8;
  private static final double ROBOT_MAX_OCCLUDED_SEC = 1.5;
  private static final double MAX_OBSERVATION_LATENCY_SEC = 0.25;
  private static final double MAX_ROBOT_SPEED_MPS = 6.5;

  private static final class PieceTrack {
    final int id;
    final String pieceId;
    final int visionClassId;
    double x, y, z;
    double vx, vy, vz;
    Rotation3d rotation;
    double posUncertainty;
    double orientConf;
    double existConf;
    double lastSeenSec;

    PieceTrack(
        int id,
        String pieceId,
        int visionClassId,
        Pose3d pose,
        double orientConf,
        double existConf,
        double ts) {
      this.id = id;
      this.pieceId = pieceId;
      this.visionClassId = visionClassId;
      this.x = pose.getX();
      this.y = pose.getY();
      this.z = pose.getZ();
      this.rotation = pose.getRotation();
      this.posUncertainty = 0.12;
      this.orientConf = orientConf;
      this.existConf = existConf;
      this.lastSeenSec = ts;
    }
  }

  private static final class RobotTrack {
    final int id;
    RobotAffiliation affiliation;
    double x, y, z;
    double lastMeasX, lastMeasY;
    double vx, vy;
    double radiusMeters;
    double muStationary; // 2-mode IMM probability of stationary/blocking mode
    double posUncertainty;
    double existConf;
    double lastSeenSec;

    RobotTrack(int id, RobotAffiliation aff, Pose3d pose, double radius, double conf, double ts) {
      this.id = id;
      this.affiliation = aff;
      this.x = pose.getX();
      this.y = pose.getY();
      this.z = pose.getZ();
      this.lastMeasX = this.x;
      this.lastMeasY = this.y;
      this.radiusMeters = radius;
      this.muStationary = 0.5;
      this.posUncertainty = 0.18;
      this.existConf = conf;
      this.lastSeenSec = ts;
    }
  }

  private final SeasonConfig season;
  private final EgoPoseBuffer egoBuffer = new EgoPoseBuffer(200);
  private final PinGuardTracker pinGuard;
  private final List<PieceTrack> pieceTracks = new ArrayList<>();
  private final List<RobotTrack> robotTracks = new ArrayList<>();

  private final Object ingestionLock = new Object();
  private final List<GamePieceObservation6DoF> pendingPieces = new ArrayList<>();
  private final List<RobotObservation> pendingRobots = new ArrayList<>();
  private final List<GamePieceObservation6DoF> scratchPieces = new ArrayList<>();
  private final List<RobotObservation> scratchRobots = new ArrayList<>();

  private int nextPieceTrackId = 1;
  private int nextRobotTrackId = 1;
  private double lastStepSec = -1.0;
  private volatile WorldSnapshot snapshot = WorldSnapshot.EMPTY;

  public FieldWorldModel(SeasonConfig season, double egoBumperRadiusMeters) {
    this.season = Objects.requireNonNull(season, "season must not be null");
    this.pinGuard = new PinGuardTracker(season.pinRules(), egoBumperRadiusMeters);
  }

  /** Pushes a new ego robot {@link Pose3d} sample (100 Hz QuestNav, odometry, or sim). */
  public void updateEgoPose(Pose3d egoPose3d, double timestampSec) {
    egoBuffer.addSample(egoPose3d, timestampSec);
  }

  /** Queues a batch of 6-DoF (or monocular-fallback) game-piece observations for the next step. */
  public void addGamePieceObservations(List<GamePieceObservation6DoF> observations) {
    if (observations != null && !observations.isEmpty()) {
      synchronized (ingestionLock) {
        pendingPieces.addAll(observations);
      }
    }
  }

  /** Queues a batch of robot observations (camera, 2D LiDAR, or fused) for the next step. */
  public void addRobotObservations(List<RobotObservation> observations) {
    if (observations != null && !observations.isEmpty()) {
      synchronized (ingestionLock) {
        pendingRobots.addAll(observations);
      }
    }
  }

  /**
   * Advances prediction, data association, 6-DoF filtering, IMM robot updates, occlusion decay, and
   * pin-guard timers to {@code nowSec}, publishing an updated immutable {@link WorldSnapshot}.
   */
  public WorldSnapshot step(double nowSec) {
    double dt = lastStepSec < 0.0 ? 0.02 : Math.max(0.0, Math.min(0.25, nowSec - lastStepSec));
    lastStepSec = nowSec;

    // 1. Predict existing tracks forward
    for (PieceTrack pt : pieceTracks) {
      pt.x += pt.vx * dt;
      pt.y += pt.vy * dt;
      pt.z = Math.max(0.0, pt.z + pt.vz * dt);
      pt.vx *= Math.exp(-dt * 2.5); // carpet friction damping
      pt.vy *= Math.exp(-dt * 2.5);
      pt.vz *= Math.exp(-dt * 4.0);
      pt.posUncertainty = Math.min(1.5, pt.posUncertainty + 0.20 * dt);
      pt.existConf *= Math.exp(-dt / PIECE_MAX_AGE_SEC);
    }

    for (RobotTrack rt : robotTracks) {
      double movingWeight = 1.0 - rt.muStationary;
      rt.x += rt.vx * movingWeight * dt;
      rt.y += rt.vy * movingWeight * dt;
      if ((nowSec - rt.lastSeenSec) > 0.06) {
        rt.vx *= Math.exp(-dt * 0.8);
        rt.vy *= Math.exp(-dt * 0.8);
      }
      rt.posUncertainty = Math.min(2.0, rt.posUncertainty + (0.45 + 0.5 * movingWeight) * dt);
      rt.existConf *= Math.exp(-dt / ROBOT_MAX_OCCLUDED_SEC);
    }

    // 2. Atomically drain cross-thread observation queues & associate
    scratchPieces.clear();
    scratchRobots.clear();
    synchronized (ingestionLock) {
      scratchPieces.addAll(pendingPieces);
      pendingPieces.clear();
      scratchRobots.addAll(pendingRobots);
      pendingRobots.clear();
    }
    processPendingPieces(nowSec);
    processPendingRobots(nowSec);

    // 4. Prune expired tracks
    pieceTracks.removeIf(p -> (nowSec - p.lastSeenSec) > PIECE_MAX_AGE_SEC || p.existConf < 0.12);
    robotTracks.removeIf(
        r -> (nowSec - r.lastSeenSec) > ROBOT_MAX_OCCLUDED_SEC || r.existConf < 0.15);

    // 5. Build immutable snapshot + pin-guard evaluation
    Pose3d egoPose = egoBuffer.latestPose().orElse(new Pose3d());
    double[] egoVel = egoBuffer.estimatePlanarVelocity();

    List<TrackedGamePiece> pieceSnap = new ArrayList<>(pieceTracks.size());
    for (PieceTrack pt : pieceTracks) {
      pieceSnap.add(
          new TrackedGamePiece(
              pt.id,
              pt.pieceId,
              pt.visionClassId,
              new Pose3d(new Translation3d(pt.x, pt.y, pt.z), pt.rotation),
              new Translation3d(pt.vx, pt.vy, pt.vz),
              pt.posUncertainty,
              pt.orientConf,
              pt.existConf,
              pt.lastSeenSec));
    }

    List<TrackedRobot> robotSnap = new ArrayList<>(robotTracks.size());
    for (RobotTrack rt : robotTracks) {
      boolean occluded = (nowSec - rt.lastSeenSec) > 0.15;
      robotSnap.add(
          new TrackedRobot(
              rt.id,
              rt.affiliation,
              new Pose3d(new Translation3d(rt.x, rt.y, rt.z), new Rotation3d()),
              rt.vx,
              rt.vy,
              rt.radiusMeters,
              rt.muStationary,
              rt.posUncertainty,
              rt.existConf,
              occluded,
              rt.lastSeenSec));
    }

    Map<Integer, OpponentPinState> pinSnap =
        pinGuard.update(egoPose.getTranslation().toTranslation2d(), robotSnap, dt);

    WorldSnapshot next =
        new WorldSnapshot(
            nowSec,
            egoPose,
            egoVel[0],
            egoVel[1],
            List.copyOf(pieceSnap),
            List.copyOf(robotSnap),
            pinSnap);
    this.snapshot = next;
    return next;
  }

  /** Returns the latest thread-safe immutable {@link WorldSnapshot}. */
  public WorldSnapshot getSnapshot() {
    return snapshot;
  }

  /** Clears all tracks, ego history, and pin timers (call at match start). */
  public void reset() {
    egoBuffer.clear();
    pinGuard.reset();
    pieceTracks.clear();
    robotTracks.clear();
    synchronized (ingestionLock) {
      pendingPieces.clear();
      pendingRobots.clear();
    }
    scratchPieces.clear();
    scratchRobots.clear();
    lastStepSec = -1.0;
    snapshot = WorldSnapshot.EMPTY;
  }

  // -------------------------------------------------------------------------
  // Internal Data Association & Filtering
  // -------------------------------------------------------------------------

  private void processPendingPieces(double nowSec) {
    if (scratchPieces.isEmpty()) {
      return;
    }
    List<GamePieceObservation6DoF> valid = new ArrayList<>(scratchPieces.size());
    List<Pose3d> fieldPoses = new ArrayList<>(scratchPieces.size());
    List<GamePieceSpec> specs = new ArrayList<>(scratchPieces.size());

    for (GamePieceObservation6DoF obs : scratchPieces) {
      if ((nowSec - obs.timestampSec()) > MAX_OBSERVATION_LATENCY_SEC) {
        continue;
      }
      Optional<GamePieceSpec> specOpt = season.findGamePieceByVisionClass(obs.visionClassId());
      if (specOpt.isEmpty()) {
        continue;
      }
      Pose3d fp = toFieldPose(obs.pose3d(), obs.frame(), obs.timestampSec());
      if (!season.isWithinFieldBounds(fp.getTranslation().toTranslation2d(), 0.3)) {
        continue;
      }
      valid.add(obs);
      fieldPoses.add(fp);
      specs.add(specOpt.get());
    }
    scratchPieces.clear();

    if (valid.isEmpty()) {
      return;
    }

    boolean[] matchedObs = new boolean[valid.size()];
    if (!pieceTracks.isEmpty()) {
      double[][] cost = new double[pieceTracks.size()][valid.size()];
      for (int r = 0; r < pieceTracks.size(); r++) {
        PieceTrack tr = pieceTracks.get(r);
        for (int c = 0; c < valid.size(); c++) {
          if (tr.visionClassId != valid.get(c).visionClassId()) {
            cost[r][c] = 1e6;
          } else {
            Pose3d fp = fieldPoses.get(c);
            double dx = tr.x - fp.getX();
            double dy = tr.y - fp.getY();
            double dz = tr.z - fp.getZ();
            cost[r][c] = Math.sqrt(dx * dx + dy * dy + dz * dz);
          }
        }
      }
      int[] assign = HungarianMatcher.solve(cost, PIECE_GATE_METERS);
      for (int r = 0; r < assign.length; r++) {
        int c = assign[r];
        if (c >= 0) {
          matchedObs[c] = true;
          updatePieceTrack(pieceTracks.get(r), valid.get(c), fieldPoses.get(c));
        }
      }
    }

    for (int c = 0; c < valid.size(); c++) {
      if (!matchedObs[c]) {
        GamePieceObservation6DoF obs = valid.get(c);
        pieceTracks.add(
            new PieceTrack(
                nextPieceTrackId++,
                specs.get(c).id(),
                obs.visionClassId(),
                fieldPoses.get(c),
                obs.hasFull6DoFOrientation() ? obs.orientationConfidence() : 0.0,
                Math.max(0.4, obs.positionConfidence()),
                obs.timestampSec()));
      }
    }
  }

  private void updatePieceTrack(PieceTrack tr, GamePieceObservation6DoF obs, Pose3d meas) {
    double dt = Math.max(0.01, obs.timestampSec() - tr.lastSeenSec);
    double alpha = Math.max(0.25, Math.min(0.85, obs.positionConfidence()));
    double ix = meas.getX() - tr.x;
    double iy = meas.getY() - tr.y;
    double iz = meas.getZ() - tr.z;

    tr.x += alpha * ix;
    tr.y += alpha * iy;
    tr.z = Math.max(0.0, tr.z + alpha * iz);
    tr.vx = 0.65 * tr.vx + 0.35 * (ix / dt);
    tr.vy = 0.65 * tr.vy + 0.35 * (iy / dt);
    tr.vz = 0.65 * tr.vz + 0.35 * (iz / dt);

    if (obs.hasFull6DoFOrientation() && obs.orientationConfidence() > 0.05) {
      double rotAlpha = Math.max(0.2, Math.min(0.85, obs.orientationConfidence()));
      tr.rotation = QuaternionSlerp.slerp(tr.rotation, meas.getRotation(), rotAlpha);
      tr.orientConf = Math.min(1.0, 0.5 * tr.orientConf + 0.5 * obs.orientationConfidence());
    }

    tr.posUncertainty = Math.max(0.03, tr.posUncertainty * (1.0 - 0.55 * alpha));
    tr.existConf = Math.min(1.0, tr.existConf + 0.35 * obs.positionConfidence());
    tr.lastSeenSec = obs.timestampSec();
  }

  private void processPendingRobots(double nowSec) {
    if (scratchRobots.isEmpty()) {
      return;
    }
    List<RobotObservation> valid = new ArrayList<>(scratchRobots.size());
    List<Pose3d> fieldPoses = new ArrayList<>(scratchRobots.size());

    for (RobotObservation obs : scratchRobots) {
      if ((nowSec - obs.timestampSec()) > MAX_OBSERVATION_LATENCY_SEC) {
        continue;
      }
      Pose3d fp = toFieldPose(obs.pose3d(), obs.frame(), obs.timestampSec());
      if (!season.isWithinFieldBounds(fp.getTranslation().toTranslation2d(), 0.5)) {
        continue;
      }
      valid.add(obs);
      fieldPoses.add(fp);
    }
    scratchRobots.clear();

    if (valid.isEmpty()) {
      return;
    }

    boolean[] matchedObs = new boolean[valid.size()];
    if (!robotTracks.isEmpty()) {
      double[][] cost = new double[robotTracks.size()][valid.size()];
      for (int r = 0; r < robotTracks.size(); r++) {
        RobotTrack tr = robotTracks.get(r);
        for (int c = 0; c < valid.size(); c++) {
          RobotObservation obs = valid.get(c);
          boolean affConflict =
              tr.affiliation != RobotAffiliation.UNKNOWN
                  && obs.affiliation() != RobotAffiliation.UNKNOWN
                  && tr.affiliation != obs.affiliation();
          if (affConflict) {
            cost[r][c] = 1e6;
          } else {
            Pose3d fp = fieldPoses.get(c);
            cost[r][c] = Math.hypot(tr.x - fp.getX(), tr.y - fp.getY());
          }
        }
      }
      int[] assign = HungarianMatcher.solve(cost, ROBOT_GATE_METERS);
      for (int r = 0; r < assign.length; r++) {
        int c = assign[r];
        if (c >= 0) {
          matchedObs[c] = true;
          updateRobotTrackImm(robotTracks.get(r), valid.get(c), fieldPoses.get(c));
        }
      }
    }

    for (int c = 0; c < valid.size(); c++) {
      if (!matchedObs[c]) {
        RobotObservation obs = valid.get(c);
        robotTracks.add(
            new RobotTrack(
                nextRobotTrackId++,
                obs.affiliation(),
                fieldPoses.get(c),
                obs.estimatedRadiusMeters(),
                Math.max(0.45, obs.confidence()),
                obs.timestampSec()));
      }
    }
  }

  private void updateRobotTrackImm(RobotTrack tr, RobotObservation obs, Pose3d meas) {
    double dt = Math.max(0.01, obs.timestampSec() - tr.lastSeenSec);
    double dx = meas.getX() - tr.x;
    double dy = meas.getY() - tr.y;
    double measVx = clampSpeed((meas.getX() - tr.lastMeasX) / dt);
    double measVy = clampSpeed((meas.getY() - tr.lastMeasY) / dt);
    tr.lastMeasX = meas.getX();
    tr.lastMeasY = meas.getY();
    double measSpeed = Math.hypot(measVx, measVy);

    // 2-mode IMM mode probability update (Stationary vs Constant-Velocity)
    double likeStat = Math.exp(-0.5 * (measSpeed / 0.35) * (measSpeed / 0.35));
    double likeMove = 1.0 - 0.7 * likeStat;
    double priorStat = 0.88 * tr.muStationary + 0.12 * (1.0 - tr.muStationary);
    double priorMove = 1.0 - priorStat;
    double norm = priorStat * likeStat + priorMove * likeMove;
    tr.muStationary = norm > 1e-9 ? (priorStat * likeStat) / norm : 0.5;

    double posGain = Math.max(0.35, Math.min(0.85, obs.confidence()));
    double velGain = (1.0 - 0.6 * tr.muStationary) * 0.45;
    tr.x += posGain * dx;
    tr.y += posGain * dy;
    tr.z = meas.getZ();
    tr.vx = clampSpeed(tr.vx + velGain * (measVx - tr.vx));
    tr.vy = clampSpeed(tr.vy + velGain * (measVy - tr.vy));

    if (tr.affiliation == RobotAffiliation.UNKNOWN
        && obs.affiliation() != RobotAffiliation.UNKNOWN) {
      tr.affiliation = obs.affiliation();
    }
    tr.radiusMeters = 0.7 * tr.radiusMeters + 0.3 * obs.estimatedRadiusMeters();
    tr.posUncertainty = Math.max(0.06, tr.posUncertainty * (1.0 - 0.5 * posGain));
    tr.existConf = Math.min(1.0, tr.existConf + 0.4 * obs.confidence());
    tr.lastSeenSec = obs.timestampSec();
  }

  private Pose3d toFieldPose(Pose3d raw, ObservationFrame frame, double timestampSec) {
    if (frame == ObservationFrame.FIELD) {
      return raw;
    }
    Pose3d egoAtTime = egoBuffer.sampleAt(timestampSec).orElse(new Pose3d());
    return egoAtTime.plus(new Transform3d(raw.getTranslation(), raw.getRotation()));
  }

  private static double clampSpeed(double v) {
    if (!Double.isFinite(v)) {
      return 0.0;
    }
    return Math.max(-MAX_ROBOT_SPEED_MPS, Math.min(MAX_ROBOT_SPEED_MPS, v));
  }
}
