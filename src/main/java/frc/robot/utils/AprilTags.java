package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swervedrive.Vision;
import java.util.List;

public final class AprilTags {

  public static final List<Integer> RED_REEF_TAGS = List.of(6, 7, 8, 9, 10, 11);
  public static final List<Integer> BLUE_REEF_TAGS = List.of(17, 18, 19, 20, 21, 22);
  public static final List<Integer> RED_STATION_TAGS = List.of(1, 2);
  public static final List<Integer> BLUE_STATION_TAGS = List.of(12, 13);

  private AprilTags() {}

  public static List<Integer> reefTags() {
    return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue
        ? BLUE_REEF_TAGS
        : RED_REEF_TAGS;
  }

  public static List<Integer> stationTags() {
    return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue
        ? BLUE_STATION_TAGS
        : RED_STATION_TAGS;
  }

  public static final Transform2d REEF_ALIGN_OFFSET = new Transform2d(0.6, 0, Rotation2d.kPi);
  public static final Transform2d STATION_ALIGN_OFFSET = new Transform2d(0.6, 0, Rotation2d.kZero);

  public static int findReefTagForAlignment(Pose2d robotPose) {
    var currentTag = -1;
    var minDistance = Double.MAX_VALUE;

    for (final var tag : AprilTags.reefTags()) {
      final var tagPose = Vision.getAprilTagPose(tag, REEF_ALIGN_OFFSET);
      final var distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());

      if (distance < minDistance) {
        currentTag = tag;
        minDistance = distance;
      }
    }

    if (currentTag < 0) {
      throw new RuntimeException("Expected to find a tag");
    }

    return currentTag;
  }

  public static int findStationTagForAlignment(Pose2d robotPose) {
    var currentTag = -1;
    var minDistance = Double.MAX_VALUE;

    for (final var tag : AprilTags.stationTags()) {
      final var tagPose = Vision.getAprilTagPose(tag, STATION_ALIGN_OFFSET);
      final var distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());

      if (distance < minDistance) {
        currentTag = tag;
        minDistance = distance;
      }
    }

    if (currentTag < 0) {
      throw new RuntimeException("Expected to find a tag");
    }

    return currentTag;
  }
}
