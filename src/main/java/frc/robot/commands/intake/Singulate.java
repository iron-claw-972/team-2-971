package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Singulator;

public class Singulate extends CommandBase{

  Singulator m_singulator;
  
  public Singulate(){
    this(Robot.singulator);
  }

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
