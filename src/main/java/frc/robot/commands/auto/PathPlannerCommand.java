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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        new PathConstraints(Constants.drive.kMaxDriveSpeed, Constants.drive.kMaxDriveAccel),
        waypoints.get(0),
        waypoints.get(1),
        (PathPoint[]) Arrays.copyOfRange(waypoints.toArray(), 2, waypoints.size())
      ));
    }

    public PathPlannerCommand(PathPlannerTrajectory path){
      this(new ArrayList<PathPlannerTrajectory>(Arrays.asList(path)), 0, Robot.drive);
    }

    public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive){
        this(PathLoader.getPathGroup(pathGroupName), pathIndex, drive); 
    }

    public PathPlannerCommand(ArrayList<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive){
        m_drive = drive;
        addRequirements(m_drive);
        if (pathIndex < 0 || pathIndex > pathGroup.size() - 1){
            throw new IndexOutOfBoundsException("Path index out of range"); 
        } 
        PathPlannerTrajectory path = pathGroup.get(pathIndex);  
        addCommands(
            (pathIndex == 0 && DriverStation.isAutonomousEnabled() ? new InstantCommand(() -> m_drive.resetOdometry(path.getInitialPose())) : new DoNothing()), 
            new PPRamseteCommand(
                path, 
                m_drive::getPose, 
                m_drive.getRamseteController(), 
                m_drive.getKinematics(), 
                m_drive::tankDriveVolts, 
                m_drive
            )
        );
        


    }
}
