package frc.robot.commands.vision;

import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.ShuffleboardManager;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SenseAprilTagAtVelocity extends CommandBase {

  Drivetrain m_drive;
  Vision m_vision; 
  ShuffleboardManager m_manager; 
  PIDController m_pidController; 
  

  /**
   * Creates a new ArcadeDrive command. It will continuously run arcade drive on the robot.
   *
   * @param drive The drivetrain subsystem used by this command.
   */
  public SenseAprilTagAtVelocity(Drivetrain drive, ShuffleboardManager manager, PIDController controller) {

    m_drive = drive;
    m_manager = manager; 
    m_pidController = controller; 

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double turn_speed = -m_pidController.calculate(m_drive.getAngularVelocityRadians(),m_manager.getAngularVelocityRadians()); // the 1 is the goal range in meters
    MathUtil.clamp(turn_speed, -1, 1); //change to (0,1) if this doesn't work
    m_drive.tankDrivePercentOutput(turn_speed,0);
    if (m_vision.hasTargets()){
      System.out.println("I see a target!");
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
