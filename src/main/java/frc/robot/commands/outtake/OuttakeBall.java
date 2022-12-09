package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Outtake;

public class OuttakeBall extends CommandBase{

  private Outtake m_outtake;
  
  public OuttakeBall(){
    this(Robot.outtake);
  }

  public OuttakeBall(Outtake outtake){
    m_outtake = outtake;
    addRequirements(m_outtake);
  }
  
  @Override
  public void execute(){
    m_outtake.outtake();
  }

  @Override
  public void end(boolean interrupted) {
    m_outtake.stop();
  }

}
