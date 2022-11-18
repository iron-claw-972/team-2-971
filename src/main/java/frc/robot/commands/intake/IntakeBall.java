package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase{

  Intake m_intake;
  
  public IntakeBall(){
    this(Robot.intake);
  }

  public IntakeBall(Intake intake){
    m_intake = intake;
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
