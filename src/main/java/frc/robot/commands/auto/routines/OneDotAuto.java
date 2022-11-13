package frc.robot.commands.auto.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;

public class OneDotAuto extends SequentialCommandGroup{
   public OneDotAuto(){
    this(Robot.drive); 
   }

   public OneDotAuto(Drivetrain drive){
       addRequirements(drive);
       addCommands(
           new PathPlannerCommand("OneDotAuto", 0, true)
       );
   }

      
}
