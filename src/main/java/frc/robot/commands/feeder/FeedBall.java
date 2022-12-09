package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Feeder;

public class FeedBall extends CommandBase{

  private Feeder m_feeder;
  private boolean m_left;
  
  public FeedBall(boolean left){
    this(Robot.feeder, left);
  }

  public FeedBall(Feeder feeder, boolean left){
    m_feeder = feeder;
    m_left = left;
    addRequirements(m_feeder);
  }
  
  @Override
  public void execute(){
    m_feeder.feed(m_left);
  }

  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
  }

}
