package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import frc.robot.util.Odometry;

public class FieldConstants {
  public final ArrayList<PathPoint> kPlant1Waypoints = new ArrayList<PathPoint>(List.of(
    Odometry.createPathPoint(0, 0, 0),
    Odometry.createPathPoint(0, 0, 0)
  ));
  public final ArrayList<PathPoint> kPlant2Waypoints = new ArrayList<PathPoint>(List.of(
    Odometry.createPathPoint(0, 0, 0),
    Odometry.createPathPoint(0, 0, 0)
  ));
  public final ArrayList<PathPoint> kPlant3Waypoints = new ArrayList<PathPoint>(List.of(
    Odometry.createPathPoint(0, 0, 0),
    Odometry.createPathPoint(0, 0, 0)
  ));
  public final ArrayList<PathPoint> kPlant4Waypoints = new ArrayList<PathPoint>(List.of(
    Odometry.createPathPoint(0, 0, 0),
    Odometry.createPathPoint(0, 0, 0)
  ));
}
