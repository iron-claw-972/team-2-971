package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class PathPlannerCommand extends SequentialCommandGroup{
    private Drivetrain m_drive; 

    public PathPlannerCommand(String trajectoryName, boolean isFirstPath, boolean stopAtEnd){
        this(trajectoryName, Robot.drive, isFirstPath, stopAtEnd);

    }

    public PathPlannerCommand(String trajectoryName, Drivetrain drive, boolean isFirstPath, boolean stopAtEnd){
        this(PathPlanner.loadPath(trajectoryName, new PathConstraints(Constants.auto.kMaxVelocity, Constants.auto.kMaxAccel)), drive, isFirstPath, stopAtEnd); 
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
