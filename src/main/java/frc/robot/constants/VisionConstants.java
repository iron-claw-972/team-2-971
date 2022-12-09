package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;



public class VisionConstants {
  
  public static final Pose3d kTargetPose = new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
  public static final Transform3d kCameraToRobot = new Transform3d(new Translation3d(x,y,z), new Rotation3d(roll, pitch, yaw)));  
}