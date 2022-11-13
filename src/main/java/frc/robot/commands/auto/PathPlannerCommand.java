package frc.robot.commands.auto;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathLoader;

public class PathPlannerCommand extends SequentialCommandGroup{
    private Drivetrain m_drive;  

    public PathPlannerCommand(String pathGroupName, int pathIndex, boolean stopAtEnd){
        this(pathGroupName, pathIndex, Robot.drive, stopAtEnd);
    }

    public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive, boolean stopAtEnd){
        this(PathLoader.getPathGroup(pathGroupName), pathIndex, drive, stopAtEnd); 
    }
    public PathPlannerCommand(ArrayList<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean stopAtEnd){
        m_drive = drive;
        addRequirements(m_drive);
        if (pathIndex < 0 || pathIndex > pathGroup.size() - 1){
            throw new IndexOutOfBoundsException("Path index out of range"); 
        } 
        PathPlannerTrajectory path = pathGroup.get(pathIndex);  
        addCommands(
            (pathIndex == 0 ? new InstantCommand(() -> m_drive.resetOdometry(path.getInitialPose())) : new DoNothing()), 
            new PPRamseteCommand(
                path, 
                m_drive::getPose, 
                m_drive.getRamseteController(), 
                m_drive.getKinematics(), 
                m_drive::tankDriveVolts, 
                m_drive
            ), 
            (stopAtEnd ? new InstantCommand(() -> m_drive.tankDriveVolts(0, 0)) : new DoNothing())
        );
        


    }
}
