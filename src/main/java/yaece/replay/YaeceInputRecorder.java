package yaece.replay;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yaece.console.PlayMode;
import yaece.json.MinimalJson;
import yaece.ledger.OptimizationObjective;
import yaece.perception.GamePieceObservation6DoF;
import yaece.perception.ObservationFrame;
import yaece.perception.RobotAffiliation;
import yaece.perception.RobotObservation;
import yaece.season.DefenseStrategyMode;
import yaece.season.MatchPeriod;

/**
 * Zero-dependency, pure-Java deterministic input recorder exporting and importing compact JSONL
 * match logs via {@link MinimalJson}.
 */
public final class YaeceInputRecorder {
  private final int maxTicks;
  private final List<RecordedTick> ticks = new ArrayList<>();

  public YaeceInputRecorder() {
    this(10_000); // > 3 minutes @ 50 Hz
  }

  public YaeceInputRecorder(int maxTicks) {
    if (maxTicks < 1) {
      throw new IllegalArgumentException("maxTicks must be >= 1");
    }
    this.maxTicks = maxTicks;
  }

  public synchronized void record(RecordedTick tick) {
    Objects.requireNonNull(tick, "tick must not be null");
    if (ticks.size() >= maxTicks) {
      ticks.remove(0);
    }
    ticks.add(tick);
  }

  public synchronized List<RecordedTick> getTicks() {
    return List.copyOf(ticks);
  }

  public synchronized void clear() {
    ticks.clear();
  }

  /** Serializes all recorded ticks to compact JSON Lines (one JSON object per 20 ms tick). */
  public synchronized String toJsonLines() {
    StringBuilder sb = new StringBuilder(ticks.size() * 256);
    for (RecordedTick t : ticks) {
      sb.append(encodeTick(t)).append('\n');
    }
    return sb.toString();
  }

  /** Writes all recorded ticks to {@code outputFile} in UTF-8 JSONL format. */
  public void writeJsonLines(Path outputFile) throws IOException {
    Files.writeString(outputFile, toJsonLines(), StandardCharsets.UTF_8);
  }

  /** Parses a UTF-8 JSONL file back into an immutable list of {@link RecordedTick}s. */
  public static List<RecordedTick> readJsonLines(Path inputFile) throws IOException {
    return parseJsonLines(Files.readString(inputFile, StandardCharsets.UTF_8));
  }

  /** Parses a JSONL string back into an immutable list of {@link RecordedTick}s. */
  public static List<RecordedTick> parseJsonLines(String jsonl) {
    if (jsonl == null || jsonl.isBlank()) {
      return List.of();
    }
    List<RecordedTick> out = new ArrayList<>();
    for (String line : jsonl.split("\\R")) {
      String trimmed = line.trim();
      if (trimmed.isEmpty()) {
        continue;
      }
      out.add(decodeTick(MinimalJson.parseObject(trimmed)));
    }
    return List.copyOf(out);
  }

  private static String encodeTick(RecordedTick t) {
    StringBuilder sb = new StringBuilder(256);
    sb.append("{\"t\":")
        .append(fmt(t.fpgaTimestampSec()))
        .append(",\"rem\":")
        .append(fmt(t.matchTimeRemainingSec()))
        .append(",\"per\":\"")
        .append(t.period().name())
        .append("\",\"pm\":\"")
        .append(t.playMode().name())
        .append("\",\"ds\":\"")
        .append(t.defenseStrategyMode().name())
        .append("\",\"obj\":\"")
        .append(t.optimizationObjective().name())
        .append("\",\"ego\":");
    appendPose3d(sb, t.egoPose3d());

    sb.append(",\"held\":{");
    boolean first = true;
    for (var e : t.heldCountsByPieceId().entrySet()) {
      if (!first) sb.append(',');
      first = false;
      sb.append('"').append(escape(e.getKey())).append("\":").append(e.getValue());
    }
    sb.append("},\"dyn\":{");
    first = true;
    for (var e : t.dynamicKeyActiveStates().entrySet()) {
      if (!first) sb.append(',');
      first = false;
      sb.append('"').append(escape(e.getKey())).append("\":").append(e.getValue());
    }
    sb.append("},\"pieces\":[");
    for (int i = 0; i < t.gamePieces().size(); i++) {
      if (i > 0) sb.append(',');
      GamePieceObservation6DoF gp = t.gamePieces().get(i);
      sb.append("{\"c\":")
          .append(gp.visionClassId())
          .append(",\"f\":\"")
          .append(gp.frame().name())
          .append("\",\"pc\":")
          .append(fmt(gp.positionConfidence()))
          .append(",\"oc\":")
          .append(fmt(gp.orientationConfidence()))
          .append(",\"full\":")
          .append(gp.hasFull6DoFOrientation())
          .append(",\"ts\":")
          .append(fmt(gp.timestampSec()))
          .append(",\"p\":");
      appendPose3d(sb, gp.pose3d());
      sb.append('}');
    }
    sb.append("],\"robots\":[");
    for (int i = 0; i < t.robots().size(); i++) {
      if (i > 0) sb.append(',');
      RobotObservation rb = t.robots().get(i);
      sb.append("{\"a\":\"")
          .append(rb.affiliation().name())
          .append("\",\"f\":\"")
          .append(rb.frame().name())
          .append("\",\"cf\":")
          .append(fmt(rb.confidence()))
          .append(",\"r\":")
          .append(fmt(rb.estimatedRadiusMeters()))
          .append(",\"ts\":")
          .append(fmt(rb.timestampSec()))
          .append(",\"p\":");
      appendPose3d(sb, rb.pose3d());
      sb.append('}');
    }
    sb.append("]}");
    return sb.toString();
  }

  private static RecordedTick decodeTick(Map<String, Object> m) {
    double t = MinimalJson.requireDouble(m, "t", "tick");
    double rem = MinimalJson.requireDouble(m, "rem", "tick");
    MatchPeriod per = MatchPeriod.valueOf(MinimalJson.requireString(m, "per", "tick"));
    PlayMode pm = PlayMode.valueOf(MinimalJson.requireString(m, "pm", "tick"));
    DefenseStrategyMode ds =
        DefenseStrategyMode.valueOf(MinimalJson.requireString(m, "ds", "tick"));
    OptimizationObjective obj =
        OptimizationObjective.valueOf(MinimalJson.requireString(m, "obj", "tick"));
    Pose3d ego = decodePose3d(MinimalJson.requireDoubleArray(m, "ego", 6, "tick"));

    Map<String, Integer> held = new LinkedHashMap<>();
    Map<String, Object> heldRaw = MinimalJson.optionalObject(m, "held", "tick");
    if (heldRaw != null) {
      for (var e : heldRaw.entrySet()) {
        held.put(e.getKey(), ((Number) e.getValue()).intValue());
      }
    }

    Map<String, Boolean> dyn = new LinkedHashMap<>();
    Map<String, Object> dynRaw = MinimalJson.optionalObject(m, "dyn", "tick");
    if (dynRaw != null) {
      for (var e : dynRaw.entrySet()) {
        dyn.put(e.getKey(), Boolean.TRUE.equals(e.getValue()));
      }
    }

    List<GamePieceObservation6DoF> pieces = new ArrayList<>();
    for (Object po : MinimalJson.optionalArray(m, "pieces", "tick")) {
      @SuppressWarnings("unchecked")
      Map<String, Object> pmObj = (Map<String, Object>) po;
      pieces.add(
          new GamePieceObservation6DoF(
              MinimalJson.requireInt(pmObj, "c", "piece"),
              decodePose3d(MinimalJson.requireDoubleArray(pmObj, "p", 6, "piece")),
              ObservationFrame.valueOf(MinimalJson.requireString(pmObj, "f", "piece")),
              MinimalJson.requireDouble(pmObj, "pc", "piece"),
              MinimalJson.requireDouble(pmObj, "oc", "piece"),
              MinimalJson.optionalBoolean(pmObj, "full", true, "piece"),
              MinimalJson.requireDouble(pmObj, "ts", "piece")));
    }

    List<RobotObservation> robots = new ArrayList<>();
    for (Object ro : MinimalJson.optionalArray(m, "robots", "tick")) {
      @SuppressWarnings("unchecked")
      Map<String, Object> rmObj = (Map<String, Object>) ro;
      robots.add(
          new RobotObservation(
              decodePose3d(MinimalJson.requireDoubleArray(rmObj, "p", 6, "robot")),
              ObservationFrame.valueOf(MinimalJson.requireString(rmObj, "f", "robot")),
              RobotAffiliation.valueOf(MinimalJson.requireString(rmObj, "a", "robot")),
              MinimalJson.requireDouble(rmObj, "cf", "robot"),
              MinimalJson.requireDouble(rmObj, "r", "robot"),
              MinimalJson.requireDouble(rmObj, "ts", "robot")));
    }

    return new RecordedTick(t, rem, per, ego, pieces, robots, held, pm, ds, obj, dyn);
  }

  private static void appendPose3d(StringBuilder sb, Pose3d p) {
    sb.append('[')
        .append(fmt(p.getX()))
        .append(',')
        .append(fmt(p.getY()))
        .append(',')
        .append(fmt(p.getZ()))
        .append(',')
        .append(fmt(p.getRotation().getX()))
        .append(',')
        .append(fmt(p.getRotation().getY()))
        .append(',')
        .append(fmt(p.getRotation().getZ()))
        .append(']');
  }

  private static Pose3d decodePose3d(double[] arr) {
    return new Pose3d(
        new Translation3d(arr[0], arr[1], arr[2]), new Rotation3d(arr[3], arr[4], arr[5]));
  }

  private static String fmt(double v) {
    return String.format(Locale.US, "%.4f", v);
  }

  private static String escape(String s) {
    return s.replace("\\", "\\\\").replace("\"", "\\\"");
  }
}
