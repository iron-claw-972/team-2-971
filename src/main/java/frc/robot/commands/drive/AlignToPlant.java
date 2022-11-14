package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Odometry;

public class AlignToPlant extends SequentialCommandGroup {

  private Drivetrain m_drive;
  private ArrayList<PathPoint> waypoints;

  public AlignToPlant(int plantNumber) {
    this(Robot.drive, plantNumber);
  }
  
  public AlignToPlant(Drivetrain drive, int plantNumber) {
    m_drive = drive;
    addRequirements(m_drive);

    waypoints.add(Odometry.getCurrentPathPoint());
    if (plantNumber == 1) {
      waypoints.addAll(Constants.field.kPlant1Waypoints);
    } else if (plantNumber == 2) {
      waypoints.addAll(Constants.field.kPlant2Waypoints);
    } else if (plantNumber == 3) {
      waypoints.addAll(Constants.field.kPlant3Waypoints);
    } else if (plantNumber == 4) {
      waypoints.addAll(Constants.field.kPlant4Waypoints);
    } else {
      throw new IllegalArgumentException("Plant number must be between 1 and 4 inclusive");
    }

    addCommands(
      new PathPlannerCommand(waypoints)
    );
  }
}
