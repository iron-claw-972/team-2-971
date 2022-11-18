package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class Intake extends CommandBase{

  Intake m_intake;
  
  public Intake(){
    this(Robot.intake);
  }

  public Intake(Intake subsystem){
    m_intake = subsystem;
    addRequirements(m_intake);
  }
  
  @Override
  public void execute(){
    m_intake.set(0.3);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

}
