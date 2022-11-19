package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase{

  private Drivetrain m_drive;

  public ArcadeDrive() {
    this(Robot.drive);
  }
  
  public ArcadeDrive(Drivetrain drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }
  
  @Override
  public void execute() {
    m_drive.arcadeDrive(Driver.getThrottleValue(), Driver.getTurnValue());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

}
