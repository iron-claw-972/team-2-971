/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;

import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.Motors;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  private final WPI_TalonFX m_rightMotor1;

  private final TalonEncoder m_leftEncoder;
  private final TalonEncoder m_rightEncoder;

  private final PhoenixMotorControllerGroup m_leftMotors;
  private final PhoenixMotorControllerGroup m_rightMotors;

  private final DifferentialDrive m_dDrive;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final PIDController m_leftDrivePID = new PIDController(
    Constants.drive.kLeftDriveP,
    Constants.drive.kLeftDriveI,
    Constants.drive.kLeftDriveD
  );
  private final PIDController m_rightDrivePID = new PIDController(
    Constants.drive.kRightDriveP,
    Constants.drive.kRightDriveI,
    Constants.drive.kRightDriveD
  );

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.drive.kTrackWidth);
  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(Constants.drive.kDriveS, Constants.drive.kDriveV);
  private final RamseteController ramseteController = new RamseteController(Constants.auto.kRamseteB, Constants.auto.kRamseteZeta); 
  
  public Drivetrain() {
    this(
      Motors.createTalonFX(Constants.drive.kLeftMotorId, NeutralMode.Brake),
      Motors.createTalonFX(Constants.drive.kRightMotorId, NeutralMode.Brake)
    );
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1) {
    m_gyro.reset();

    // Motor setup
    m_leftMotor1 = leftMotor1;
    m_rightMotor1 = rightMotor1;

    m_leftMotors = new PhoenixMotorControllerGroup(m_leftMotor1);
    m_rightMotors = new PhoenixMotorControllerGroup(m_rightMotor1);

    m_leftMotors.setInverted(true); // clockwise
    m_rightMotors.setInverted(false); // counterclockwise

    // Encoder setup
    m_leftEncoder = new TalonEncoder(m_leftMotor1);
    m_rightEncoder = new TalonEncoder(m_rightMotor1);

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);

    resetEncoders();

    // Drivetrain setup
    m_dDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // Odometry setup
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public RamseteController getRamseteController(){
    return ramseteController; 
  }

  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics; 
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculate FF
    final double leftFeedforward = m_driveFF.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_driveFF.calculate(speeds.rightMetersPerSecond);

    // Calculate PID
    final double leftOutput = m_leftDrivePID.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightDrivePID.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    // Output PID+FF
    m_leftMotors.setVoltage(leftOutput + leftFeedforward);
    m_rightMotors.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double throttle, double turn) {
    // Convert drivetrain throttle and turn to left and right wheel speeds
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(throttle, 0.0, turn));

    setSpeeds(wheelSpeeds);
  }

  public void updateOdometry() {
    // Upate robot pose (x, y, theta)
    m_odometry.update(
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
    );
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void arcadeDrive(double throttle, double turn){
    m_dDrive.arcadeDrive(throttle, turn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_dDrive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());

  }


}
