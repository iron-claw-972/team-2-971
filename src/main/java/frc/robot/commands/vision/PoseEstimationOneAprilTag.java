package frc.robot.commands.vision;

import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.ShuffleboardManager;

import com.kauailabs.navx.frc.AHRS;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseEstimationOneAprilTag extends CommandBase {

  Vision m_vision; 
  AHRS m_ahrs;  
  DifferentialDrivePoseEstimator m_estimator;
  Drivetrain m_drive;
  /**
   * Creates a new ArcadeDrive command. It will continuously run arcade drive on the robot.
   *
   * @param drive The drivetrain subsystem used by this command.
   */
  public PoseEstimationOneAprilTag(Vision vision, AHRS ahrs, Drivetrain drive) {
    m_vision = vision; 
    m_ahrs = ahrs; 
    m_drive = drive; 
    
    addRequirements(vision);//do I addRequirements for the drivetrain as well??
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
  //lots of help from here: https://www.youtube.com/watch?v=TG9KAa2EGzQ    
    m_estimator.update(m_ahrs.getRotation2d(), m_drive.getWheelSpeeds(), m_drive.getLeftWheelDistanceSinceOdometryReset(), m_drive.getRightWheelDistanceSinceOdometryReset());
    var res = m_vision.returnLatestResult();
    if (res.hasTargets()) {
        var target = res.getBestTarget(); 
        double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
        Transform3d camToTargetTrans = res.getBestTarget().getBestCameraToTarget();//do we have target sort mode stuff set up in Photnvision UI?
        Pose3d camPose = Constants.vision.kTargetPose.transformBy(camToTargetTrans.inverse());
        var visionMeasurement = camPose.transformBy(Constants.vision.kCameraToRobot); 
        m_estimator.addVisionMeasurement(
                visionMeasurement.toPose2d(), imageCaptureTime);
    }

  }

  @Override
  public void end(boolean interrupted) {
      
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
