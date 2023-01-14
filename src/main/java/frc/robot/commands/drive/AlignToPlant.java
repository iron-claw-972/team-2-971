package frc.robot.commands.drive;

import java.util.ArrayList;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import lib.Odometry;

public class AlignToPlant extends SequentialCommandGroup {

  private Drivetrain m_drive;
  private ArrayList<PathPoint> waypoints = new ArrayList<PathPoint>();

  public AlignToPlant(boolean topPlants) {
    this(Robot.drive, topPlants);
  }
  
  public AlignToPlant(Drivetrain drive, boolean topPlants) {
    m_drive = drive;
    addRequirements(m_drive);

    waypoints.add(Odometry.getCurrentPathPoint());
    if (topPlants) {
      waypoints.addAll(Constants.field.kTopPlantsWaypoints);
    } else {
      waypoints.addAll(Constants.field.kBottomPlantsWaypoints);
    }

    addCommands(
      new PathPlannerCommand(waypoints)
    );
  }
}
