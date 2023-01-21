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
import ctre_shims.TalonEncoderSim;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.Vision;
import lib.Motors;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  private final WPI_TalonFX m_rightMotor1;

  private final TalonEncoder m_leftEncoder;
  private final TalonEncoder m_rightEncoder;

  private final PhoenixMotorControllerGroup m_leftMotors;
  private final PhoenixMotorControllerGroup m_rightMotors;

  private final DifferentialDrive m_dDrive;

  private final AHRS m_gyro;

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
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  private final SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(Constants.drive.kSLinear, Constants.drive.kVLinear);
  private final RamseteController ramseteController = new RamseteController(Constants.auto.kRamseteB, Constants.auto.kRamseteZeta); 

  private final Field2d m_field = new Field2d();

  // Simulation
  private final TalonEncoderSim m_leftEncoderSim;
  private final TalonEncoderSim m_rightEncoderSim;
  private final LinearSystem<N2, N2, N2> m_driveSystem = LinearSystemId.identifyDrivetrainSystem(
    Constants.drive.kVLinear,
    Constants.drive.kALinear,
    Constants.drive.kVAngular,
    Constants.drive.kAAngular
  );
  private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    m_driveSystem,
    Constants.drive.kGearbox,
    Constants.drive.kGearRatio,
    Constants.drive.kTrackWidth,
    Constants.drive.kWheelRadius,
    null
  );
  
  public Drivetrain() {
    this(
      Motors.createTalonFX(Constants.drive.kLeftMotorId, NeutralMode.Brake, true, 30, 40, 1),
      Motors.createTalonFX(Constants.drive.kRightMotorId, NeutralMode.Brake, true, 30, 40, 1)
    );
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1) {
    this(leftMotor1, rightMotor1, new AHRS(SPI.Port.kMXP), new DifferentialDrive(leftMotor1, rightMotor1));
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1, AHRS gyro) {
    this(leftMotor1, rightMotor1, gyro, new DifferentialDrive(leftMotor1, rightMotor1));
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1, DifferentialDrive dDrive) {
    this(leftMotor1, rightMotor1, new AHRS(SPI.Port.kMXP), dDrive);
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX rightMotor1, AHRS gyro, DifferentialDrive dDrive) {
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

    // Encoder setup
    m_leftEncoder = new TalonEncoder(m_leftMotor1);
    m_rightEncoder = new TalonEncoder(m_rightMotor1);

    m_leftEncoderSim = new TalonEncoderSim(m_leftEncoder);
    m_rightEncoderSim = new TalonEncoderSim(m_rightEncoder);

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kGearRatio / Constants.drive.kEncoderResolution);

    resetEncoders();

    // Gyro setup
    m_gyro = gyro;
    resetGyro();

    // Drivetrain setup
    m_dDrive = dDrive;
    SmartDashboard.putData(m_dDrive);

    // Odometry setup
    AprilTagFieldLayout aprilTagFieldLayout = Vision.getTagFieldLayout();
    Optional<Pose3d> tag2 = aprilTagFieldLayout.getTagPose(2);
        

    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d(new Translation2d(15.05, 2.79), new Rotation2d(0)));

    // Place field on Shuffleboard
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(
      m_leftMotors.get() * RobotController.getBatteryVoltage(),
      m_rightMotors.get() * RobotController.getBatteryVoltage()
    );
    m_driveSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());

    // NavX Sim
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-m_driveSim.getHeading().getDegrees(), 360));
  }

  public RamseteController getRamseteController(){
    return ramseteController; 
  }

  public SimpleMotorFeedforward getDriveFF() {
    return m_driveFF;
  }

  public PIDController getLeftDrivePID() {
    return m_leftDrivePID;
  }

  public PIDController getRightDrivePID() {
    return m_rightDrivePID;
  }

  public void resetGyro() {
    m_gyro.reset();
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
    m_dDrive.feed(); 
  }

  public void feedForwardDrive(double throttle, double turn) {
    // Convert drivetrain throttle and turn to left and right wheel speeds
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(throttle, 0.0, turn));

    setSpeeds(wheelSpeeds);
  }

  public void updateOdometry() {
    // Upate robot pose (x, y, theta)
    m_poseEstimator.update(
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
    );

    Optional<Pair<Pose3d,Double>> result = Vision.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent() && result.get().getFirst() != null && result.get().getSecond() != null) {
      Pair<Pose3d,Double> camPose = result.get();
      m_poseEstimator.addVisionMeasurement(camPose.getFirst().toPose2d(), Timer.getFPGATimestamp() - Units.millisecondsToSeconds(camPose.getSecond()));
      // m_poseEstimator.addVisionMeasurement(new Pose2d(), 0.02);
      // System.out.println(camPose.getFirst().toPose2d().toString());
    }
  }

  public void printPose(){
    Pose2d p = m_poseEstimator.getEstimatedPosition();
    System.out.println(Vision.getTagFieldLayout().getTagPose(2).toString());
    System.out.printf("ROBOT POSE:\ntoString(): %s\nRotation: %.2f degrees\nPosition: (%.2f, %.2f)\n", p.toString(), p.getRotation().getDegrees(), p.getX(), p.getY());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_leftMotor1.setSelectedSensorPosition(0); 
    m_rightMotor1.setSelectedSensorPosition(0); 
    m_driveSim.setPose(pose);
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftWheelDistanceSinceOdometryReset(){
    return m_leftMotor1.getSelectedSensorPosition()/2048/62*8*2*Math.PI*2;
  }

  public double getRightWheelDistanceSinceOdometryReset(){
    return m_rightMotor1.getSelectedSensorPosition()/2048/62*8*2*Math.PI*2;
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
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
