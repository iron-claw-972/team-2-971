package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.Controller;

public class ArcadeDrive extends CommandBase{

  Drivetrain m_drive = new Drivetrain();
  
    public ArcadeDrive(Drivetrain subsystem){
      m_drive = subsystem;
      addRequirements(m_drive);
    }
    
    @Override
    public void execute(){
        m_drive.arcadeDrive(Driver.getThrottleValue(), Driver.getTurnValue());
    }

    @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

}
