package frc.robot.commands.auto;

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

    public PathPlannerCommand(String trajectoryName, boolean isFirstPath, boolean stopAtEnd){
        this(trajectoryName, Robot.drive, isFirstPath, stopAtEnd);

    }

    public PathPlannerCommand(String trajectoryName, Drivetrain drive, boolean isFirstPath, boolean stopAtEnd){
        this(PathLoader.getTrajectory(trajectoryName), drive, isFirstPath, stopAtEnd); 
    }
    public PathPlannerCommand(PathPlannerTrajectory trajectory, Drivetrain drive, boolean isFirstPath, boolean stopAtEnd){
        m_drive = drive;
        addRequirements(m_drive);
        addCommands(
            (isFirstPath ? new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())) : new DoNothing()), 
            new PPRamseteCommand(
                trajectory, 
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
