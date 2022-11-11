package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class FFDrive extends CommandBase{

  Drivetrain m_drive;
  
  public FFDrive(Drivetrain drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }
  
  @Override
  public void execute() {
    double throttle = Driver.getThrottleValue() * Constants.drive.kMaxDriveSpeed;
    double turn = Driver.getTurnValue() * Constants.drive.kMaxTurnSpeed;

    m_drive.drive(throttle, turn);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0);
  }

}
