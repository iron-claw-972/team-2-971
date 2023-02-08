package frc.robot.commands;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class TestCameraPose extends CommandBase{
  private Drivetrain m_drive;
  private Pose2d currentPose;
  private int timer = 200;
  private double speed;
  private Pose2d[] poses;
  private ArrayList<Pose2d> robotPoses;

  public TestCameraPose(double speed){
    this(speed, Robot.drive);
  }

  public TestCameraPose(double speed, Drivetrain drive){
    addRequirements(drive);
    this.speed=speed;
  }

  private Pose2d getPose(){
    Optional<Pair<Pose3d, Double>> p = Vision.getEstimatedGlobalPose(currentPose);
    if(p.isPresent() && p.get().getFirst() != null && p.get().getSecond() != null && p.get().getFirst().getX() > -10000 && p.get().getSecond() >= 0){
      return p.get().getFirst().toPose2d();
    }
    return null;
  }

  @Override
  public void initialize(){
    currentPose=getPose();
    poses = new Pose2d[timer];
    robotPoses = new ArrayList<Pose2d>(0);
  }

  @Override
  public void execute(){
    m_drive.arcadeDrive(0, speed);
    timer--;
    if(getPose()!=null&&timer>=0){
      poses[timer]=getPose();
      currentPose=getPose();
    }
  }

  private double findDistance(Pose2d pose1, Pose2d pose2){
    return Math.sqrt(Math.pow(pose1.getX()-pose2.getX(), 2) + Math.pow(pose1.getY()-pose2.getY(), 2));
  }

  private Pose2d findMidpoint(Pose2d pose1, Pose2d pose2){
    return new Pose2d(pose1.getX()/2+pose2.getX()/2, pose1.getY()/2+pose2.getY()/2, new Rotation2d(0));
  }

  private Pose2d calculatePose(Pose2d pose1, Pose2d pose2){
    double side1 = findDistance(pose1, pose2);
    double angle1 = Math.abs(pose1.getRotation().getRadians()-pose2.getRotation().getRadians());
    if(angle1>Math.PI){
      angle1=2*Math.PI-angle1;
    }
    double angle2 = 90-angle1/2;
    double side2 = side1*Math.sin(angle2)/Math.sin(angle1);

    Pose2d pose1Direction1 = new Pose2d(pose1.getX()-pose1.getRotation().getCos()*side2, pose1.getY()-pose1.getRotation().getSin(), new Rotation2d(0));
    Pose2d pose2Direction1 = new Pose2d(pose2.getX()-pose2.getRotation().getCos()*side2, pose2.getY()-pose2.getRotation().getSin(), new Rotation2d(0));
    Pose2d pose1Direction2 = new Pose2d(pose1.getX()+pose1.getRotation().getCos()*side2, pose1.getY()+pose1.getRotation().getSin(), new Rotation2d(0));
    Pose2d pose2Direction2 = new Pose2d(pose2.getX()+pose2.getRotation().getCos()*side2, pose2.getY()+pose2.getRotation().getSin(), new Rotation2d(0));

    if(findDistance(pose1Direction1, pose2Direction1)<findDistance(pose1Direction2, pose2Direction2)){
      return findMidpoint(pose1Direction1, pose2Direction1);
    }else{
      return findMidpoint(pose1Direction2, pose2Direction2);
    }
  }

  @Override
  public void end(boolean interrupted){
    m_drive.arcadeDrive(0, 0);

    if(interrupted){
      System.out.println("The test was interrupted. Run it again.");
      return;
    }

    for(int i = 0; i < poses.length; i++){
      for(int j = 0; j < poses.length; j++){
        if(poses[i]!=null&&poses[j]!=null){
          double angle = Math.abs(poses[i].getRotation().getDegrees()-poses[j].getRotation().getDegrees());
          if(angle>180){
            angle = 360-angle;
          }
          if(angle>10&&angle<85){
            robotPoses.add(calculatePose(poses[i], poses[j]));
          }
        }
      }
    }

    double totalX = 0;
    double totalY = 0;
    for(int i = 0; i < robotPoses.size(); i++){
      totalX += robotPoses.get(i).getX();
      totalY += robotPoses.get(i).getY();
    }
    Pose2d averagePose = new Pose2d(totalX/robotPoses.size(), totalY/robotPoses.size(), new Rotation2d(0));

    double totalDist = 0;
    double minDist = 10000;
    double maxDist = 0;
    int poseNumber=0;
    for(int i = 0; i < poses.length; i++){
      if(poses[i]!=null){
        poseNumber++;
        double d = findDistance(poses[1], averagePose);
        totalDist+=d;
        if(d<minDist){
          minDist=d;
        }
        if(d>maxDist){
          maxDist=d;
        }
      }
    }
    double averageDist=totalDist/poseNumber;
    System.out.printf("\nTest results:\nAverage distance from actual pose to calculated pose: %.5f\nLowest distance: %.5f\nHighest distance: %.5f\n%s\n", averageDist, minDist, maxDist, maxDist-minDist>0.04?"Cover one of the cameras and try again":averageDist>0.02?"Remeasure the robot to camera distances":"The robot to camera distances are probably good");
  }

  @Override
  public boolean isFinished(){
    return timer <= 0;
  }
}
