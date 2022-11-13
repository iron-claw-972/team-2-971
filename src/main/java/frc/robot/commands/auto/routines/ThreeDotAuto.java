package frc.robot.commands.auto.routines;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathLoader;

public class ThreeDotAuto extends SequentialCommandGroup{
    
    public ThreeDotAuto(){
        this(Robot.drive); 
    }

    public ThreeDotAuto(Drivetrain drive){
       addRequirements(drive);
       addCommands(
           new PathPlannerCommand("ThreeDotAuto", 0, true)
       );
   }

      
}
