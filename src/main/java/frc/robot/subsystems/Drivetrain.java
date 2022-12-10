/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import lib.Motors;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX m_leftMotor1;
  private final WPI_TalonSRX m_rightMotor1;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final PhoenixMotorControllerGroup m_leftMotors;
  private final PhoenixMotorControllerGroup m_rightMotors;

  private final DifferentialDrive m_dDrive;

  private final DifferentialDriveOdometry m_odometry; 

  private final AHRS m_gyro;


  public Drivetrain() {
    this(
      Motors.createTalonSRX(Constants.drive.kLeftMotorId, NeutralMode.Brake, true, true, 30, 40, 1),
      Motors.createTalonSRX(Constants.drive.kRightMotorId, NeutralMode.Brake, true, true, 30, 40, 1)
    );
  }

  public Drivetrain(WPI_TalonSRX leftMotor1, WPI_TalonSRX rightMotor1) {
    this(leftMotor1, rightMotor1, new AHRS(SPI.Port.kMXP), new DifferentialDrive(leftMotor1, rightMotor1));
  }

  public Drivetrain(WPI_TalonSRX leftMotor1, WPI_TalonSRX rightMotor1, AHRS gyro) {
    this(leftMotor1, rightMotor1, gyro, new DifferentialDrive(leftMotor1, rightMotor1));
  }

  public Drivetrain(WPI_TalonSRX leftMotor1, WPI_TalonSRX rightMotor1, DifferentialDrive dDrive) {
    this(leftMotor1, rightMotor1, new AHRS(SPI.Port.kMXP), dDrive);
  }

  public Drivetrain(WPI_TalonSRX leftMotor1, WPI_TalonSRX rightMotor1, AHRS gyro, DifferentialDrive dDrive) {
    // Motor setup
    m_leftMotor1 = leftMotor1;
    m_rightMotor1 = rightMotor1;

    m_leftMotors = new PhoenixMotorControllerGroup(m_leftMotor1);
    m_rightMotors = new PhoenixMotorControllerGroup(m_rightMotor1);

    if (RobotBase.isSimulation()) {
      m_leftMotors.setInverted(false);
      m_rightMotors.setInverted(false);
    } else {
      m_leftMotors.setInverted(false);
      m_rightMotors.setInverted(true);
    }

    m_leftEncoder = new Encoder(3, 4, true, Encoder.EncodingType.k4X);
    m_rightEncoder = new Encoder(1, 2, false, Encoder.EncodingType.k4X); 
    // // Encoder setup
    // m_leftEncoder = new TalonEncoder(m_leftMotor1);
    // m_rightEncoder = new TalonEncoder(m_rightMotor1);

   
    // m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);
    // m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);

    resetEncoders();

    // Gyro setup
    m_gyro = gyro;
    resetGyro();

    // Drivetrain setup
    m_dDrive = dDrive;
    SmartDashboard.putData(m_dDrive);

    // Odometry setup
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }


  @Override
  public void periodic() {
    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
   
  }

 
  public void resetGyro() {
    m_gyro.reset();
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
    m_leftMotor1.setSelectedSensorPosition(0); 
    m_rightMotor1.setSelectedSensorPosition(0); 
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftWheelDistanceSinceOdometryReset(){
    return (m_leftEncoder.getDistance()/2050)*6*Math.PI;  
  }

  public double getRightWheelDistanceSinceOdometryReset(){
    return (m_rightEncoder.getDistance()/2050)*6*Math.PI;  

  }

  public void drive(double leftSpeed, double rightSpeed){
    m_leftMotor1.set(ControlMode.PercentOutput, leftSpeed);
    m_rightMotor1.set(ControlMode.PercentOutput, rightSpeed); 
  }
  



  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public double getPoseX() {
    return getPose().getX();
  }

  public double getPoseY() {
    return getPose().getY();
  }

  public double getPoseRotation() {
    return getPose().getRotation().getDegrees();
  }

  public void arcadeDrive(double throttle, double turn){
    m_dDrive.arcadeDrive(throttle, turn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_dDrive.feed();
  }

  public void tankDrivePercentOutput(double leftPower, double rightPower){
    m_leftMotor1.set(ControlMode.PercentOutput, 0.25);
    m_rightMotor1.set(ControlMode.PercentOutput, 0.25);
  }

  public double getDriveSpeed(){
    return (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2;
  }

  public double getGyroRotation(){
    return m_gyro.getRotation2d().getDegrees();
  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public double getAngularVelocityRadians(){
    return m_gyro.getRate()*(Math.PI/180); 

  }
}
