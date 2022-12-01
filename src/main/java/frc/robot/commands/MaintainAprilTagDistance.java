package frc.robot.commands;

import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MaintainAprilTagDistance extends CommandBase {

  PhotonPipelineResult m_result;
  Drivetrain m_drive; 
  PIDController m_pidController; 
  double forwardSpeed; 


  /**
   * Creates a new ArcadeDrive command. It will continuously run arcade drive on the robot.
   *
   * @param drive The drivetrain subsystem used by this command.
   */
  public MaintainAprilTagDistance(PhotonPipelineResult result, Drivetrain drive, PIDController pidController) {

    // store the drivetrain in the m_drive variable to be accessed elsewhere
    m_result = result;
    m_drive = drive;
    m_pidController = pidController; 

    // add the drivetrain as a requirement so the scheduler 
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_result.hasTargets()){
        double range =
        PhotonUtils.calculateDistanceToTargetMeters(
                //fill in these values
                Units.inchesToMeters(5.125), 
                Units.inchesToMeters(15.5), // if this doesn't work put in the actual values in meters
                0,//PROBABLY NOT
                Units.degreesToRadians(m_result.getBestTarget().getPitch()));

    forwardSpeed = -m_pidController.calculate(range, 1); // the 1 is the goal range in meters
    MathUtil.clamp(forwardSpeed, -0.2, 0.2);
    // -1.0 required to ensure positive PID controller effort _increases_ range

    }

    m_drive.tankDrivePercentOutput(forwardSpeed, forwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
