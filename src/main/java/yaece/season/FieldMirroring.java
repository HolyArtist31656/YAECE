// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.season;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/** Pure mathematical utility for mirroring canonical Blue-origin WPILib geometry to Red. */
public final class FieldMirroring {
  private static final Rotation3d ROT_Z_PI = new Rotation3d(0.0, 0.0, Math.PI);

  private FieldMirroring() {}

  public static Translation2d mirror(
      Translation2d blue,
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    Objects.requireNonNull(blue, "blue translation must not be null");
    Objects.requireNonNull(alliance, "alliance must not be null");
    Objects.requireNonNull(mode, "mirror mode must not be null");
    if (alliance == AllianceSide.BLUE || mode == FieldMirrorMode.NONE) {
      return blue;
    }
    return switch (mode) {
      case ROTATIONAL_180 ->
          new Translation2d(fieldLengthMeters - blue.getX(), fieldWidthMeters - blue.getY());
      case REFLECTION_X -> new Translation2d(fieldLengthMeters - blue.getX(), blue.getY());
      case NONE -> blue;
    };
  }

  public static Translation3d mirror(
      Translation3d blue,
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    Objects.requireNonNull(blue, "blue translation must not be null");
    Objects.requireNonNull(alliance, "alliance must not be null");
    Objects.requireNonNull(mode, "mirror mode must not be null");
    if (alliance == AllianceSide.BLUE || mode == FieldMirrorMode.NONE) {
      return blue;
    }
    return switch (mode) {
      case ROTATIONAL_180 ->
          new Translation3d(
              fieldLengthMeters - blue.getX(), fieldWidthMeters - blue.getY(), blue.getZ());
      case REFLECTION_X ->
          new Translation3d(fieldLengthMeters - blue.getX(), blue.getY(), blue.getZ());
      case NONE -> blue;
    };
  }

  public static Rotation2d mirror(Rotation2d blue, AllianceSide alliance, FieldMirrorMode mode) {
    Objects.requireNonNull(blue, "blue rotation must not be null");
    Objects.requireNonNull(alliance, "alliance must not be null");
    Objects.requireNonNull(mode, "mirror mode must not be null");
    if (alliance == AllianceSide.BLUE || mode == FieldMirrorMode.NONE) {
      return blue;
    }
    return switch (mode) {
      case ROTATIONAL_180 -> blue.plus(Rotation2d.fromRadians(Math.PI));
      case REFLECTION_X -> new Rotation2d(-blue.getCos(), blue.getSin());
      case NONE -> blue;
    };
  }

  public static Rotation3d mirror(Rotation3d blue, AllianceSide alliance, FieldMirrorMode mode) {
    Objects.requireNonNull(blue, "blue rotation must not be null");
    Objects.requireNonNull(alliance, "alliance must not be null");
    Objects.requireNonNull(mode, "mirror mode must not be null");
    if (alliance == AllianceSide.BLUE || mode == FieldMirrorMode.NONE) {
      return blue;
    }
    return switch (mode) {
      case ROTATIONAL_180 -> blue.rotateBy(ROT_Z_PI);
      case REFLECTION_X -> {
        double roll = -blue.getX();
        double pitch = blue.getY();
        double yaw = Math.atan2(Math.sin(blue.getZ()), -Math.cos(blue.getZ()));
        yield new Rotation3d(roll, pitch, yaw);
      }
      case NONE -> blue;
    };
  }

  public static Pose2d mirror(
      Pose2d blue,
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    Objects.requireNonNull(blue, "blue pose must not be null");
    return new Pose2d(
        mirror(blue.getTranslation(), alliance, fieldLengthMeters, fieldWidthMeters, mode),
        mirror(blue.getRotation(), alliance, mode));
  }

  public static Pose3d mirror(
      Pose3d blue,
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    Objects.requireNonNull(blue, "blue pose must not be null");
    return new Pose3d(
        mirror(blue.getTranslation(), alliance, fieldLengthMeters, fieldWidthMeters, mode),
        mirror(blue.getRotation(), alliance, mode));
  }

  public static List<Translation2d> mirrorPolygon(
      List<Translation2d> blueVertices,
      AllianceSide alliance,
      double fieldLengthMeters,
      double fieldWidthMeters,
      FieldMirrorMode mode) {
    Objects.requireNonNull(blueVertices, "blueVertices must not be null");
    if (alliance == AllianceSide.BLUE || mode == FieldMirrorMode.NONE) {
      return List.copyOf(blueVertices);
    }
    return blueVertices.stream()
        .map(v -> mirror(v, alliance, fieldLengthMeters, fieldWidthMeters, mode))
        .toList();
  }

  /** Even-odd ray-casting point-in-polygon test in 2D field coordinates. */
  public static boolean polygonContainsPoint(List<Translation2d> polygon, Translation2d point) {
    Objects.requireNonNull(polygon, "polygon must not be null");
    Objects.requireNonNull(point, "point must not be null");
    boolean inside = false;
    int n = polygon.size();
    double px = point.getX();
    double py = point.getY();
    for (int i = 0, j = n - 1; i < n; j = i++) {
      double xi = polygon.get(i).getX();
      double yi = polygon.get(i).getY();
      double xj = polygon.get(j).getX();
      double yj = polygon.get(j).getY();
      boolean intersects =
          ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
      if (intersects) {
        inside = !inside;
      }
    }
    return inside;
  }
}
