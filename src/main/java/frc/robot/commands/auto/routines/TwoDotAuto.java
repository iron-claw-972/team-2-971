package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;

public class TwoDotAuto extends SequentialCommandGroup{
    
    public TwoDotAuto(){
        this(Robot.drive); 
    }

   public TwoDotAuto(Drivetrain drive){
       addRequirements(drive);
       addCommands(
           new PathPlannerCommand("TwoDotAuto", 0)
       );
   }
}
