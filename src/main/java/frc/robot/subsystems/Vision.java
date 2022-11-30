/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  private final PhotonCamera m_camera;
  private PhotonPipelineResult m_latestResult;
  private PhotonTrackedTarget m_bestTarget;
  private double m_targetYaw = -1;
  private boolean m_hasTargets = false;
  private double m_targetId = -1;
  private double m_targetDistance = -1;
  private double m_latency = -1;

  public Vision() {
    this(new PhotonCamera("Microsoft_LifeCam_HD-3000"));
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
      m_targetDistance = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(4.5), Units.inchesToMeters(42), 0, Units.degreesToRadians(m_bestTarget.getPitch()));
      m_latency = m_latestResult.getLatencyMillis();
    }
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

  public double getTargetDistance() {
    return m_targetDistance;
  }

  public double getLatency() {
    return m_latency;
  }
 
}
