package frc.robot.util;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class Odometry {
  public static PathPoint getCurrentPathPoint() {
    return poseToPathPoint(Robot.drive.getPose());
  }

 public static PathPoint poseToPathPoint(Pose2d pose) {
   return new PathPoint(pose.getTranslation(), pose.getRotation());
 }

 public static PathPoint createPathPoint(double x, double y, double theta, double velocityOverride) {
   return new PathPoint(new Translation2d(x, y), new Rotation2d(Units.degreesToRadians(theta)), velocityOverride);
 }

 public static PathPoint createPathPoint(double x, double y, double theta) {
   return new PathPoint(new Translation2d(x, y), new Rotation2d(Units.degreesToRadians(theta)));
 }
}
