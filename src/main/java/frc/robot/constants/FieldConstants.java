package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import lib.Odometry;

public class FieldConstants {
  public final ArrayList<PathPoint> kTopPlantsWaypoints = new ArrayList<PathPoint>(List.of(
    Odometry.createPathPoint(0.61, 4.59, 180)
  ));
  public final ArrayList<PathPoint> kBottomPlantsWaypoints = new ArrayList<PathPoint>(List.of(
    Odometry.createPathPoint(0.61, 1.51, 180)
  ));
}
