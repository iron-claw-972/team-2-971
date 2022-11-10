package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Singulator;
import lib.controllers.Controller;

public class Singulate extends CommandBase{

  Singulator m_singulator = new Singulator();
  
    public Singulate(Singulator subsystem){
      m_singulator = subsystem;
      addRequirements(m_singulator);
    }
    
    @Override
    public void execute(){
      m_singulator.set(0.3);
    }

    @Override
  public void end(boolean interrupted) {
    m_singulator.stop();
  }

}
