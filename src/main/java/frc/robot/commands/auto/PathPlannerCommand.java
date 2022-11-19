package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathLoader;

public class PathPlannerCommand extends SequentialCommandGroup{
    private Drivetrain m_drive;  

    public PathPlannerCommand(String pathGroupName, int pathIndex){
        this(pathGroupName, pathIndex, Robot.drive);
    }

    public PathPlannerCommand(ArrayList<PathPoint> waypoints) {
      this(PathPlanner.generatePath(
        new PathConstraints(Constants.auto.kMaxAutoSpeed, Constants.auto.kMaxAutoAccel),
        waypoints.get(0),
        waypoints.get(1),
        (PathPoint[]) Arrays.copyOfRange(waypoints.toArray(), 2, waypoints.size())
      ));
    }

    public PathPlannerCommand(PathPlannerTrajectory path){
      this(new ArrayList<PathPlannerTrajectory>(Arrays.asList(path)), 0, Robot.drive, false);
    }

    public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive){
        this(PathLoader.getPathGroup(pathGroupName), pathIndex, drive, true); 
    }

    public PathPlannerCommand(ArrayList<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose){
        m_drive = drive;
        addRequirements(m_drive);
        if (pathIndex < 0 || pathIndex > pathGroup.size() - 1){
            throw new IndexOutOfBoundsException("Path index out of range"); 
        } 
        PathPlannerTrajectory path = pathGroup.get(pathIndex);  
        addCommands(
            (pathIndex == 0 && resetPose && RobotBase.isReal() ? new InstantCommand(() -> m_drive.resetOdometry(path.getInitialPose())) : new DoNothing()),
            new PPRamseteCommand(
                path,
                m_drive::getPose, 
                m_drive.getRamseteController(),
                m_drive.getDriveFF(),
                m_drive.getKinematics(),
                m_drive::getWheelSpeeds,
                m_drive.getLeftDrivePID(),
                m_drive.getRightDrivePID(),
                m_drive::tankDriveVolts,
                m_drive
            )
        );
        


    }
}
