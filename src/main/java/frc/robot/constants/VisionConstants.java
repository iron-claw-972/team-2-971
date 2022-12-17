package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;



public class VisionConstants {
  public static final String kCameraName = "Global_Shutter_Camera";
  public static final Transform2d kCameraToRobot = new Transform2d(new Translation2d(0, 0), new Rotation2d(0)); 
  public static final Map<Integer, Pose2d> kTagPoses = Map.ofEntries(
    Map.entry(0, new Pose2d()),
    Map.entry(1, new Pose2d())
  );
}