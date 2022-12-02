package frc.robot.commands.vision;

import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.ShuffleboardManager;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SenseAprilTagAtVelocity extends CommandBase {

  Drivetrain m_drive;
  Vision m_vision; 
  PhotonCamera m_camera; 
  ShuffleboardManager m_manager; 

  /**
   * Creates a new ArcadeDrive command. It will continuously run arcade drive on the robot.
   *
   * @param drive The drivetrain subsystem used by this command.
   */
  public SenseAprilTagAtVelocity(Drivetrain drive, ShuffleboardManager manager, PhotonCamera camera) {

    m_drive = drive;
    m_manager = manager; 
    m_camera = camera; 

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_manager.getAngularVelocity();
    
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
