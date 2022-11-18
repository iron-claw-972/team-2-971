package frc.robot.commands.auto.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoTest extends SequentialCommandGroup{
   public AutoTest(){
    this(Robot.drive); 
   }

   public AutoTest(Drivetrain drive){
       addRequirements(drive);
       addCommands(
           new PathPlannerCommand("AutoTest", 0)
       );
   }

      
}
