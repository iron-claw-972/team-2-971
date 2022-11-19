package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;

public class ThreeDotAuto extends SequentialCommandGroup{
    
    public ThreeDotAuto(){
        this(Robot.drive); 
    }

    public ThreeDotAuto(Drivetrain drive){
       addRequirements(drive);
       addCommands(
           new PathPlannerCommand("ThreeDotAuto", 0)
       );
   }

      
}
