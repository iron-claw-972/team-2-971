/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;



public class Vision extends SubsystemBase {
  private final PhotonCamera m_camera;
  private PhotonPipelineResult m_latestResult;
  private PhotonTrackedTarget m_bestTarget;
  private double m_targetYaw = -1;
  private boolean m_hasTargets = false;
  private double m_targetId = -1;
  private double m_latency = -1;
 

  public Vision() {
    this(new PhotonCamera(Constants.vision.kCameraName));
  }

  public Vision(PhotonCamera camera) {
    m_camera = camera;
  }

  @Override
  public void periodic() {
    m_latestResult = m_camera.getLatestResult();
    m_hasTargets = m_latestResult.hasTargets();
    if (m_hasTargets) {
      m_bestTarget = m_latestResult.getBestTarget();
      m_targetYaw = m_bestTarget.getYaw();
      m_targetId = m_bestTarget.getFiducialId();
      m_latency = m_latestResult.getLatencyMillis();
    }
  }

  public Map<Pose2d, Double> getPoseEstimation() {
    if (m_hasTargets) {
      double imageCaptureTime = Timer.getFPGATimestamp() - m_latency;
      Transform2d camToTargetTrans = m_bestTarget.getBestCameraToTarget();
      Pose2d camPose = Constants.vision.kTagPoses.get(m_targetId).transformBy(camToTargetTrans.inverse());
      return Collections.singletonMap(camPose.transformBy(Constants.vision.kCameraToRobot), imageCaptureTime);
    }
    return null;
  }

  public PhotonPipelineResult returnLatestResult(){
    return m_latestResult;
  }

  public boolean hasTargets() {
    return m_hasTargets;
  }

  public double getTargetYaw() {
    return m_targetYaw;
  }

  public double getTargetId() {
    return m_targetId;
  }

  public double getLatency() {
    return m_latency;
  }

 
}
