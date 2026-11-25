// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import yaece.json.JsonParseException;
import yaece.json.MinimalJson;

/**
 * Immutable, validated root configuration for an FRC season in YAECE.
 *
 * <p>Externalizes field dimensions, alliance symmetry, game-piece 3D geometry and inventory caps,
 * Auto/Teleop scoring site point values, linked bonus/RP objectives, FRC pin rules, restricted
 * zones, and defensive strategy defaults.
 */
public final class SeasonConfig {
  public static final int SUPPORTED_SCHEMA_VERSION = 1;

  private final int schemaVersion;
  private final String seasonName;
  private final double fieldLengthMeters;
  private final double fieldWidthMeters;
  private final FieldMirrorMode mirrorMode;
  private final int maxTotalHeldGamePieces;
  private final double matchDurationSeconds;
  private final List<GamePieceSpec> gamePieces;
  private final List<ScoringSiteSpec> scoringSites;
  private final List<StaticObstacleSpec> staticObstacles;
  private final PinRuleSpec pinRules;
  private final List<RestrictedZoneSpec> restrictedZones;
  private final List<ScoringObjectiveSpec> scoringObjectives;
  private final DefenseConfigSpec defenseDefaults;
  private final List<ScoringIntervalSpec> scoringIntervals;

  private final Map<String, GamePieceSpec> pieceById;
  private final Map<Integer, GamePieceSpec> pieceByVisionClassId;
  private final Map<String, ScoringSiteSpec> siteById;
  private final Map<String, StaticObstacleSpec> obstacleById;
  private final Map<String, RestrictedZoneSpec> restrictedZoneById;
  private final Map<String, ScoringObjectiveSpec> objectiveById;
  private final Map<AllianceSide, Map<String, List<Translation2d>>> obstaclePolyCache =
      new EnumMap<>(AllianceSide.class);
  private final Map<AllianceSide, Map<String, List<Translation2d>>> zonePolyCache =
      new EnumMap<>(AllianceSide.class);
  private final Map<AllianceSide, Map<String, List<Translation2d>>> defensePolyCache =
      new EnumMap<>(AllianceSide.class);

  public SeasonConfig(
      int schemaVersion,
      String seasonName,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mirrorMode,
      int maxTotalHeldGamePieces,
      double matchDurationSeconds,
      List<GamePieceSpec> gamePieces,
      List<ScoringSiteSpec> scoringSites,
      List<StaticObstacleSpec> staticObstacles,
      PinRuleSpec pinRules,
      List<RestrictedZoneSpec> restrictedZones,
      List<ScoringObjectiveSpec> scoringObjectives,
      DefenseConfigSpec defenseDefaults,
      List<ScoringIntervalSpec> scoringIntervals) {
    this.schemaVersion = schemaVersion;
    this.seasonName = seasonName;
    this.fieldLengthMeters = fieldLengthMeters;
    this.fieldWidthMeters = fieldWidthMeters;
    this.mirrorMode = mirrorMode;
    this.maxTotalHeldGamePieces = maxTotalHeldGamePieces;
    this.matchDurationSeconds = matchDurationSeconds;
    this.gamePieces = gamePieces == null ? List.of() : List.copyOf(gamePieces);
    this.scoringSites = scoringSites == null ? List.of() : List.copyOf(scoringSites);
    this.staticObstacles = staticObstacles == null ? List.of() : List.copyOf(staticObstacles);
    this.pinRules = pinRules == null ? PinRuleSpec.FRC_DEFAULT : pinRules;
    this.restrictedZones = restrictedZones == null ? List.of() : List.copyOf(restrictedZones);
    this.scoringObjectives = scoringObjectives == null ? List.of() : List.copyOf(scoringObjectives);
    this.defenseDefaults = defenseDefaults == null ? DefenseConfigSpec.DEFAULT : defenseDefaults;
    this.scoringIntervals = scoringIntervals == null ? List.of() : List.copyOf(scoringIntervals);
    for (AllianceSide side : AllianceSide.values()) {
      Map<String, List<Translation2d>> obsMap = new HashMap<>();
      for (StaticObstacleSpec obs : this.staticObstacles) {
        obsMap.put(
            obs.id(),
            List.copyOf(obs.verticesFor(side, fieldLengthMeters, fieldWidthMeters, mirrorMode)));
      }
      obstaclePolyCache.put(side, Map.copyOf(obsMap));

      Map<String, List<Translation2d>> zMap = new HashMap<>();
      for (RestrictedZoneSpec z : this.restrictedZones) {
        zMap.put(
            z.id(),
            List.copyOf(z.verticesFor(side, fieldLengthMeters, fieldWidthMeters, mirrorMode)));
      }
      zonePolyCache.put(side, Map.copyOf(zMap));

      Map<String, List<Translation2d>> dMap = new HashMap<>();
      for (DefenseZoneSpec dz : this.defenseDefaults.defenseZones()) {
        dMap.put(
            dz.id(),
            List.copyOf(
                FieldMirroring.mirrorPolygon(
                    dz.blueVertices(), side, fieldLengthMeters, fieldWidthMeters, mirrorMode)));
      }
      defensePolyCache.put(side, Map.copyOf(dMap));
    }

    List<String> violations = new ArrayList<>();
    Map<String, GamePieceSpec> pById = new LinkedHashMap<>();
    Map<Integer, GamePieceSpec> pByClass = new HashMap<>();
    Map<String, ScoringSiteSpec> sById = new LinkedHashMap<>();
    Map<String, StaticObstacleSpec> oById = new LinkedHashMap<>();
    Map<String, RestrictedZoneSpec> zById = new LinkedHashMap<>();
    Map<String, ScoringObjectiveSpec> objById = new LinkedHashMap<>();

    validateInto(violations, pById, pByClass, sById, oById, zById, objById);
    if (!violations.isEmpty()) {
      throw new SeasonConfigException(violations);
    }

    this.pieceById = Collections.unmodifiableMap(pById);
    this.pieceByVisionClassId = Collections.unmodifiableMap(pByClass);
    this.siteById = Collections.unmodifiableMap(sById);
    this.obstacleById = Collections.unmodifiableMap(oById);
    this.restrictedZoneById = Collections.unmodifiableMap(zById);
    this.objectiveById = Collections.unmodifiableMap(objById);
  }

  public static SeasonConfig fromJsonString(String json) {
    try {
      return fromJsonMap(MinimalJson.parseObject(json));
    } catch (JsonParseException | IllegalArgumentException ex) {
      throw new SeasonConfigException(ex.getMessage(), ex);
    }
  }

  public static SeasonConfig fromFile(Path path) {
    Objects.requireNonNull(path, "path must not be null");
    try {
      return fromJsonString(Files.readString(path, StandardCharsets.UTF_8));
    } catch (IOException ex) {
      throw new SeasonConfigException("Failed to read season config file: " + path, ex);
    }
  }

  public static SeasonConfig fromClasspathResource(String resourcePath) {
    Objects.requireNonNull(resourcePath, "resourcePath must not be null");
    try (InputStream in = SeasonConfig.class.getResourceAsStream(resourcePath)) {
      if (in == null) {
        throw new SeasonConfigException("Classpath season resource not found: " + resourcePath);
      }
      return fromJsonString(new String(in.readAllBytes(), StandardCharsets.UTF_8));
    } catch (IOException ex) {
      throw new SeasonConfigException(
          "Failed to read classpath season resource: " + resourcePath, ex);
    }
  }

  public int schemaVersion() {
    return schemaVersion;
  }

  public String seasonName() {
    return seasonName;
  }

  public double fieldLengthMeters() {
    return fieldLengthMeters;
  }

  public double fieldWidthMeters() {
    return fieldWidthMeters;
  }

  public FieldMirrorMode mirrorMode() {
    return mirrorMode;
  }

  public int maxTotalHeldGamePieces() {
    return maxTotalHeldGamePieces;
  }

  public double matchDurationSeconds() {
    return matchDurationSeconds;
  }

  public List<GamePieceSpec> gamePieces() {
    return gamePieces;
  }

  public List<ScoringSiteSpec> scoringSites() {
    return scoringSites;
  }

  public List<StaticObstacleSpec> staticObstacles() {
    return staticObstacles;
  }

  public PinRuleSpec pinRules() {
    return pinRules;
  }

  public List<RestrictedZoneSpec> restrictedZones() {
    return restrictedZones;
  }

  public List<ScoringObjectiveSpec> scoringObjectives() {
    return scoringObjectives;
  }

  public DefenseConfigSpec defenseDefaults() {
    return defenseDefaults;
  }

  public List<ScoringIntervalSpec> scoringIntervals() {
    return scoringIntervals;
  }

  public Optional<GamePieceSpec> findGamePiece(String id) {
    return Optional.ofNullable(pieceById.get(id));
  }

  public Optional<GamePieceSpec> findGamePieceByVisionClass(int visionClassId) {
    return Optional.ofNullable(pieceByVisionClassId.get(visionClassId));
  }

  public Optional<ScoringSiteSpec> findScoringSite(String id) {
    return Optional.ofNullable(siteById.get(id));
  }

  public Optional<StaticObstacleSpec> findStaticObstacle(String id) {
    return Optional.ofNullable(obstacleById.get(id));
  }

  public Optional<RestrictedZoneSpec> findRestrictedZone(String id) {
    return Optional.ofNullable(restrictedZoneById.get(id));
  }

  public Optional<ScoringObjectiveSpec> findScoringObjective(String id) {
    return Optional.ofNullable(objectiveById.get(id));
  }

  public boolean isWithinFieldBounds(Translation2d point, double marginMeters) {
    Objects.requireNonNull(point, "point must not be null");
    return point.getX() >= -marginMeters
        && point.getX() <= fieldLengthMeters + marginMeters
        && point.getY() >= -marginMeters
        && point.getY() <= fieldWidthMeters + marginMeters;
  }

  /**
   * Returns cached, immutable mirrored vertices for {@code obs} on {@code alliance} (zero 50 Hz
   * allocation).
   */
  public List<Translation2d> resolvedObstaclePolygon(
      StaticObstacleSpec obs, AllianceSide alliance) {
    List<Translation2d> cached = obstaclePolyCache.get(alliance).get(obs.id());
    return cached != null
        ? cached
        : obs.verticesFor(alliance, fieldLengthMeters, fieldWidthMeters, mirrorMode);
  }

  /**
   * Returns cached, immutable mirrored vertices for {@code zone} on {@code alliance} (zero 50 Hz
   * allocation).
   */
  public List<Translation2d> resolvedZonePolygon(RestrictedZoneSpec zone, AllianceSide alliance) {
    List<Translation2d> cached = zonePolyCache.get(alliance).get(zone.id());
    return cached != null
        ? cached
        : zone.verticesFor(alliance, fieldLengthMeters, fieldWidthMeters, mirrorMode);
  }

  /**
   * Returns cached, immutable mirrored vertices for {@code dz} on {@code alliance} (zero 50 Hz
   * allocation).
   */
  public List<Translation2d> resolvedDefenseZonePolygon(DefenseZoneSpec dz, AllianceSide alliance) {
    List<Translation2d> cached = defensePolyCache.get(alliance).get(dz.id());
    return cached != null
        ? cached
        : FieldMirroring.mirrorPolygon(
            dz.blueVertices(), alliance, fieldLengthMeters, fieldWidthMeters, mirrorMode);
  }

  public Pose3d resolveSitePose(ScoringSiteSpec site, AllianceSide alliance) {
    return site.poseFor(alliance, fieldLengthMeters, fieldWidthMeters, mirrorMode);
  }

  public Pose3d resolveSiteApproachPose(ScoringSiteSpec site, AllianceSide alliance) {
    return site.approachPoseFor(alliance, fieldLengthMeters, fieldWidthMeters, mirrorMode);
  }

  public Pose2d resolveSiteStagingPose(ScoringSiteSpec site, AllianceSide alliance) {
    return site.stagingPoseFor(alliance, fieldLengthMeters, fieldWidthMeters, mirrorMode);
  }

  public List<ScoringSiteSpec> scoringSitesForPiece(String pieceId) {
    return scoringSites.stream().filter(s -> s.acceptsPiece(pieceId)).toList();
  }

  /**
   * Returns the nominal average Teleop point value for a game-piece type across sites that accept
   * it (used when estimating opponent points from observed scored piece counts).
   */
  public double nominalTeleopPointsForPiece(String pieceId) {
    List<ScoringSiteSpec> sites = scoringSitesForPiece(pieceId);
    if (sites.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (ScoringSiteSpec s : sites) {
      sum += s.teleopPoints();
    }
    return sum / sites.size();
  }

  private void validateInto(
      List<String> violations,
      Map<String, GamePieceSpec> pById,
      Map<Integer, GamePieceSpec> pByClass,
      Map<String, ScoringSiteSpec> sById,
      Map<String, StaticObstacleSpec> oById,
      Map<String, RestrictedZoneSpec> zById,
      Map<String, ScoringObjectiveSpec> objById) {
    if (schemaVersion != SUPPORTED_SCHEMA_VERSION) {
      violations.add(
          "Unsupported schemaVersion "
              + schemaVersion
              + " (expected "
              + SUPPORTED_SCHEMA_VERSION
              + ")");
    }
    if (seasonName == null || seasonName.isBlank()) {
      violations.add("seasonName must be non-blank");
    }
    if (!Double.isFinite(fieldLengthMeters) || fieldLengthMeters <= 0.0) {
      violations.add("fieldLengthMeters must be finite and > 0");
    }
    if (!Double.isFinite(fieldWidthMeters) || fieldWidthMeters <= 0.0) {
      violations.add("fieldWidthMeters must be finite and > 0");
    }
    if (mirrorMode == null) {
      violations.add("mirrorMode must not be null");
    }
    if (maxTotalHeldGamePieces < 1) {
      violations.add("maxTotalHeldGamePieces must be >= 1");
    }
    if (!Double.isFinite(matchDurationSeconds) || matchDurationSeconds <= 0.0) {
      violations.add("matchDurationSeconds must be finite and > 0");
    }
    if (gamePieces.isEmpty()) {
      violations.add("gamePieces must contain at least 1 game piece");
    }

    for (GamePieceSpec piece : gamePieces) {
      if (pById.put(piece.id(), piece) != null) {
        violations.add("Duplicate gamePiece id: '" + piece.id() + "'");
      }
      if (pByClass.put(piece.visionClassId(), piece) != null) {
        violations.add("Duplicate gamePiece visionClassId: " + piece.visionClassId());
      }
      if (piece.maxHeldSimultaneously() > maxTotalHeldGamePieces) {
        violations.add(
            "gamePiece '"
                + piece.id()
                + "' maxHeldSimultaneously ("
                + piece.maxHeldSimultaneously()
                + ") exceeds maxTotalHeldGamePieces ("
                + maxTotalHeldGamePieces
                + ")");
      }
    }

    for (ScoringSiteSpec site : scoringSites) {
      if (sById.put(site.id(), site) != null) {
        violations.add("Duplicate scoringSite id: '" + site.id() + "'");
      }
      for (String accepted : site.acceptedPieceIds()) {
        if (!pById.containsKey(accepted)) {
          violations.add(
              "scoringSite '" + site.id() + "' references unknown gamePiece id '" + accepted + "'");
        }
      }
      if (Double.isFinite(fieldLengthMeters)
          && fieldLengthMeters > 0.0
          && Double.isFinite(fieldWidthMeters)
          && fieldWidthMeters > 0.0) {
        Translation2d siteXy = site.bluePose3d().getTranslation().toTranslation2d();
        if (!isWithinFieldBounds(siteXy, 0.5)) {
          violations.add(
              "scoringSite '" + site.id() + "' bluePose3d XY " + siteXy + " is outside bounds");
        }
        Translation2d stagingXy = site.stagingHoldPose2d().getTranslation();
        if (!isWithinFieldBounds(stagingXy, 0.0)) {
          violations.add(
              "scoringSite '"
                  + site.id()
                  + "' stagingHoldPose2d XY "
                  + stagingXy
                  + " is outside bounds");
        }
      }
    }

    for (StaticObstacleSpec obs : staticObstacles) {
      if (oById.put(obs.id(), obs) != null) {
        violations.add("Duplicate staticObstacle id: '" + obs.id() + "'");
      }
    }

    for (RestrictedZoneSpec zone : restrictedZones) {
      if (zById.put(zone.id(), zone) != null) {
        violations.add("Duplicate restrictedZone id: '" + zone.id() + "'");
      }
    }

    for (ScoringObjectiveSpec obj : scoringObjectives) {
      if (objById.put(obj.id(), obj) != null) {
        violations.add("Duplicate scoringObjective id: '" + obj.id() + "'");
      }
      for (String siteId : obj.matchingSiteIds()) {
        if (!sById.containsKey(siteId)) {
          violations.add(
              "scoringObjective '"
                  + obj.id()
                  + "' references unknown scoringSite '"
                  + siteId
                  + "'");
        }
      }
      for (String pieceId : obj.matchingPieceIds()) {
        if (!pById.containsKey(pieceId)) {
          violations.add(
              "scoringObjective '" + obj.id() + "' references unknown gamePiece '" + pieceId + "'");
        }
      }
    }
  }

  private static SeasonConfig fromJsonMap(Map<String, Object> root) {
    int schemaVersion = MinimalJson.requireInt(root, "schemaVersion", "root");
    String seasonName = MinimalJson.requireString(root, "seasonName", "root");
    double fieldLength = MinimalJson.requireDouble(root, "fieldLengthMeters", "root");
    double fieldWidth = MinimalJson.requireDouble(root, "fieldWidthMeters", "root");
    FieldMirrorMode mirrorMode =
        parseEnum(
            FieldMirrorMode.class,
            MinimalJson.requireString(root, "mirrorMode", "root"),
            "root.mirrorMode");
    int maxTotalHeld = MinimalJson.requireInt(root, "maxTotalHeldGamePieces", "root");
    double matchDurationSec =
        MinimalJson.optionalDouble(root, "matchDurationSeconds", 150.0, "root");

    List<Object> rawPieces = MinimalJson.requireArray(root, "gamePieces", "root");
    List<GamePieceSpec> pieces = new ArrayList<>(rawPieces.size());
    for (int i = 0; i < rawPieces.size(); i++) {
      String ctx = "gamePieces[" + i + "]";
      Map<String, Object> obj = MinimalJson.asObject(rawPieces.get(i), ctx);
      double[] dims = MinimalJson.requireDoubleArray(obj, "dimensionsMeters", 3, ctx);
      pieces.add(
          new GamePieceSpec(
              MinimalJson.requireString(obj, "id", ctx),
              MinimalJson.requireInt(obj, "visionClassId", ctx),
              dims[0],
              dims[1],
              dims[2],
              MinimalJson.requireDouble(obj, "restHeightMeters", ctx),
              parseEnum(
                  GamePieceSymmetry.class,
                  MinimalJson.requireString(obj, "symmetry", ctx),
                  ctx + ".symmetry"),
              MinimalJson.requireInt(obj, "maxHeldSimultaneously", ctx),
              MinimalJson.optionalBoolean(obj, "requireOrientationForIntake", false, ctx),
              MinimalJson.optionalDouble(obj, "maxIntakeYawErrorDeg", 180.0, ctx),
              MinimalJson.optionalDouble(obj, "maxIntakeTiltErrorDeg", 45.0, ctx)));
    }

    List<Object> rawSites = MinimalJson.optionalArray(root, "scoringSites", "root");
    List<ScoringSiteSpec> sites = new ArrayList<>(rawSites.size());
    for (int i = 0; i < rawSites.size(); i++) {
      String ctx = "scoringSites[" + i + "]";
      Map<String, Object> obj = MinimalJson.asObject(rawSites.get(i), ctx);
      double[] poseArr = MinimalJson.requireDoubleArray(obj, "pose3d", 6, ctx);
      Pose3d bluePose =
          new Pose3d(
              new Translation3d(poseArr[0], poseArr[1], poseArr[2]),
              new Rotation3d(poseArr[3], poseArr[4], poseArr[5]));

      double[] offArr = MinimalJson.requireDoubleArray(obj, "approachOffsetMeters", 6, ctx);
      Transform3d approachOffset =
          new Transform3d(
              new Translation3d(offArr[0], offArr[1], offArr[2]),
              new Rotation3d(offArr[3], offArr[4], offArr[5]));

      Set<String> accepted =
          new HashSet<>(MinimalJson.optionalStringList(obj, "acceptedPieces", ctx));
      if (accepted.isEmpty()) {
        throw new JsonParseException(ctx + ".acceptedPieces must be a non-empty string array");
      }

      // Support either legacy "points" OR separate "autoPoints" / "teleopPoints"
      double fallbackPts = MinimalJson.optionalDouble(obj, "points", -1.0, ctx);
      double autoPts =
          obj.containsKey("autoPoints")
              ? MinimalJson.requireDouble(obj, "autoPoints", ctx)
              : fallbackPts;
      double teleopPts =
          obj.containsKey("teleopPoints")
              ? MinimalJson.requireDouble(obj, "teleopPoints", ctx)
              : fallbackPts;
      if (autoPts < 0.0 || teleopPts < 0.0) {
        throw new JsonParseException(
            ctx + " must specify either 'points' or both 'autoPoints' and 'teleopPoints'");
      }

      double[] holdArr = MinimalJson.requireDoubleArray(obj, "stagingHoldPose2d", 3, ctx);
      Pose2d stagingPose = new Pose2d(holdArr[0], holdArr[1], Rotation2d.fromRadians(holdArr[2]));

      Set<String> tags = new HashSet<>(MinimalJson.optionalStringList(obj, "tags", ctx));

      sites.add(
          new ScoringSiteSpec(
              MinimalJson.requireString(obj, "id", ctx),
              bluePose,
              approachOffset,
              accepted,
              autoPts,
              teleopPts,
              MinimalJson.optionalInt(obj, "siteCapacity", -1, ctx),
              stagingPose,
              tags,
              MinimalJson.optionalString(obj, "dynamicAvailabilityKey", "", ctx)));
    }

    List<Object> rawObstacles = MinimalJson.optionalArray(root, "staticObstacles", "root");
    List<StaticObstacleSpec> obstacles = new ArrayList<>(rawObstacles.size());
    for (int i = 0; i < rawObstacles.size(); i++) {
      String ctx = "staticObstacles[" + i + "]";
      Map<String, Object> obj = MinimalJson.asObject(rawObstacles.get(i), ctx);
      obstacles.add(
          new StaticObstacleSpec(
              MinimalJson.requireString(obj, "id", ctx),
              parsePolygon(obj, "polygon2d", ctx),
              MinimalJson.optionalDouble(obj, "heightMeters", 2.0, ctx),
              MinimalJson.optionalBoolean(obj, "blocksVision", true, ctx),
              MinimalJson.optionalBoolean(obj, "blocksChassis", true, ctx)));
    }

    PinRuleSpec pinRules = PinRuleSpec.FRC_DEFAULT;
    Map<String, Object> rawPin = MinimalJson.optionalObject(root, "pinRules", "root");
    if (rawPin != null) {
      pinRules =
          new PinRuleSpec(
              MinimalJson.optionalDouble(rawPin, "maxPinSeconds", 3.0, "pinRules"),
              MinimalJson.optionalDouble(rawPin, "backoffTriggerSeconds", 2.2, "pinRules"),
              MinimalJson.optionalDouble(rawPin, "separationDistanceMeters", 1.83, "pinRules"),
              MinimalJson.optionalDouble(rawPin, "cooldownHoldSeconds", 3.2, "pinRules"));
    }

    List<Object> rawZones = MinimalJson.optionalArray(root, "restrictedZones", "root");
    List<RestrictedZoneSpec> zones = new ArrayList<>(rawZones.size());
    for (int i = 0; i < rawZones.size(); i++) {
      String ctx = "restrictedZones[" + i + "]";
      Map<String, Object> obj = MinimalJson.asObject(rawZones.get(i), ctx);
      zones.add(
          new RestrictedZoneSpec(
              MinimalJson.requireString(obj, "id", ctx),
              parseEnum(
                  ZoneRuleType.class,
                  MinimalJson.requireString(obj, "ruleType", ctx),
                  ctx + ".ruleType"),
              parsePolygon(obj, "polygon2d", ctx),
              MinimalJson.optionalBoolean(obj, "activeInAuto", true, ctx),
              MinimalJson.optionalBoolean(obj, "activeInTeleop", true, ctx),
              MinimalJson.optionalDouble(
                  obj, "activeWhenMatchTimeRemainingSecLessOrEqual", -1.0, ctx),
              MinimalJson.optionalDouble(obj, "safetyBufferMeters", 0.45, ctx)));
    }

    List<Object> rawObjectives = MinimalJson.optionalArray(root, "scoringObjectives", "root");
    List<ScoringObjectiveSpec> objectives = new ArrayList<>(rawObjectives.size());
    for (int i = 0; i < rawObjectives.size(); i++) {
      String ctx = "scoringObjectives[" + i + "]";
      Map<String, Object> obj = MinimalJson.asObject(rawObjectives.get(i), ctx);
      objectives.add(
          new ScoringObjectiveSpec(
              MinimalJson.requireString(obj, "id", ctx),
              parseEnum(
                  ObjectiveRewardType.class,
                  MinimalJson.requireString(obj, "rewardType", ctx),
                  ctx + ".rewardType"),
              MinimalJson.requireDouble(obj, "rewardValue", ctx),
              parseEnum(
                  ObjectiveAggregationMode.class,
                  MinimalJson.requireString(obj, "aggregationMode", ctx),
                  ctx + ".aggregationMode"),
              MinimalJson.requireInt(obj, "targetCount", ctx),
              new HashSet<>(MinimalJson.optionalStringList(obj, "matchingSiteIds", ctx)),
              new HashSet<>(MinimalJson.optionalStringList(obj, "matchingSiteTags", ctx)),
              new HashSet<>(MinimalJson.optionalStringList(obj, "matchingPieceIds", ctx))));
    }

    DefenseConfigSpec defenseDefaults = DefenseConfigSpec.DEFAULT;
    Map<String, Object> rawDef = MinimalJson.optionalObject(root, "defenseDefaults", "root");
    if (rawDef != null) {
      DefenseStrategyMode pref =
          parseEnum(
              DefenseStrategyMode.class,
              MinimalJson.optionalString(
                  rawDef, "defaultPreferredMode", "HIGHEST_THREAT", "defenseDefaults"),
              "defenseDefaults.defaultPreferredMode");
      double leadMargin =
          MinimalJson.optionalDouble(
              rawDef, "zoneDefenseLeadMarginPoints", 12.0, "defenseDefaults");
      double trailMargin =
          MinimalJson.optionalDouble(
              rawDef, "highestThreatTrailMarginPoints", 6.0, "defenseDefaults");
      List<Object> rawDefZones =
          MinimalJson.optionalArray(rawDef, "defenseZones", "defenseDefaults");
      List<DefenseZoneSpec> defZones = new ArrayList<>(rawDefZones.size());
      for (int i = 0; i < rawDefZones.size(); i++) {
        String ctx = "defenseDefaults.defenseZones[" + i + "]";
        Map<String, Object> zObj = MinimalJson.asObject(rawDefZones.get(i), ctx);
        double[] anchorArr = MinimalJson.requireDoubleArray(zObj, "anchorPose2d", 3, ctx);
        defZones.add(
            new DefenseZoneSpec(
                MinimalJson.requireString(zObj, "id", ctx),
                parsePolygon(zObj, "polygon2d", ctx),
                new Pose2d(anchorArr[0], anchorArr[1], Rotation2d.fromRadians(anchorArr[2])),
                MinimalJson.optionalDouble(zObj, "priorityWeight", 1.0, ctx)));
      }
      defenseDefaults = new DefenseConfigSpec(pref, leadMargin, trailMargin, defZones);
    }

    List<Object> rawIntervals = MinimalJson.optionalArray(root, "scoringIntervals", "root");
    List<ScoringIntervalSpec> intervals = new ArrayList<>(rawIntervals.size());
    for (int i = 0; i < rawIntervals.size(); i++) {
      String ctx = "scoringIntervals[" + i + "]";
      Map<String, Object> iObj = MinimalJson.asObject(rawIntervals.get(i), ctx);
      intervals.add(
          new ScoringIntervalSpec(
              MinimalJson.requireString(iObj, "id", ctx),
              MinimalJson.requireDouble(iObj, "startElapsedSec", ctx),
              MinimalJson.requireDouble(iObj, "endElapsedSec", ctx),
              new HashSet<>(MinimalJson.optionalStringList(iObj, "matchingSiteIds", ctx)),
              new HashSet<>(MinimalJson.optionalStringList(iObj, "matchingSiteTags", ctx)),
              MinimalJson.optionalBoolean(iObj, "active", true, ctx),
              MinimalJson.optionalDouble(iObj, "pointMultiplier", 1.0, ctx),
              MinimalJson.optionalDouble(iObj, "flatBonusPoints", 0.0, ctx)));
    }

    return new SeasonConfig(
        schemaVersion,
        seasonName,
        fieldLength,
        fieldWidth,
        mirrorMode,
        maxTotalHeld,
        matchDurationSec,
        pieces,
        sites,
        obstacles,
        pinRules,
        zones,
        objectives,
        defenseDefaults,
        intervals);
  }

  private static List<Translation2d> parsePolygon(Map<String, Object> obj, String key, String ctx) {
    List<Object> rawVerts = MinimalJson.requireArray(obj, key, ctx);
    List<Translation2d> vertices = new ArrayList<>(rawVerts.size());
    for (int j = 0; j < rawVerts.size(); j++) {
      Object vObj = rawVerts.get(j);
      if (!(vObj instanceof List<?> pair) || pair.size() != 2) {
        throw new JsonParseException(
            ctx + "." + key + "[" + j + "] must be a 2-element [x, y] array");
      }
      double x = MinimalJson.asFiniteDouble(pair.get(0), ctx + "." + key + "[" + j + "][0]");
      double y = MinimalJson.asFiniteDouble(pair.get(1), ctx + "." + key + "[" + j + "][1]");
      vertices.add(new Translation2d(x, y));
    }
    return vertices;
  }

  private static <E extends Enum<E>> E parseEnum(Class<E> enumType, String value, String path) {
    try {
      return Enum.valueOf(enumType, value);
    } catch (IllegalArgumentException ex) {
      throw new JsonParseException(
          "Invalid value '"
              + value
              + "' for "
              + path
              + " (expected "
              + enumType.getSimpleName()
              + ")");
    }
  }
}
