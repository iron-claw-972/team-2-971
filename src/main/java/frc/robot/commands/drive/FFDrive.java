package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class FFDrive extends CommandBase{

  private Drivetrain m_drive;

  public FFDrive() {
    this(Robot.drive);
  }
  
  public FFDrive(Drivetrain drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }
  
  @Override
  public void execute() {
    double throttle = -Driver.getThrottleValue() * Constants.drive.kMaxDriveSpeed;
    double turn = Driver.getTurnValue() * Constants.drive.kMaxTurnSpeed;

    m_drive.feedForwardDrive(throttle, turn);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.feedForwardDrive(0, 0);
  }

}
