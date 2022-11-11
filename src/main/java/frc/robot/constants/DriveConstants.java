package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public final double kMaxDriveSpeed = 3.0; // m/s
  public final double kMaxDriveAccel = 3.0; // m/s^2
  public final double kMaxTurnSpeed = 2 * Math.PI; // rot/s

  public final double kTrackWidth = Units.inchesToMeters(24.79); // Distance from left wheels to right wheels
  public final double kWheelRadius = Units.inchesToMeters(2); // Wheel radius
  public final double kEncoderResolution = 2048; // 2048 ticks per rotation
  public final double kGearRatio = (double) 62 / 8; // # driven gear teeth / # driver gear teeth
  public final DCMotor kGearbox = DCMotor.getFalcon500(1);

  public final double kDriveSlewRate = 3; // 1/X seconds from 0 to 1
  public final double kTurnSlewRate = 3; // 1/X seconds from 0 to 1

  public final double kDeadband = 0.02; // don't run motors when less than X power commanded

  public final int kLeftMotorId = 16; // fx8
  public final int kRightMotorId = 34; // fx17

  // Left drive PID
  public final double kLeftDriveP = 1;
  public final double kLeftDriveI = 1;
  public final double kLeftDriveD = 1;

  // Right drive PID
  public final double kRightDriveP = kLeftDriveP;
  public final double kRightDriveI = kLeftDriveI;
  public final double kRightDriveD = kLeftDriveD;

  // Drive feedforward
  public final double kSLinear = 1;
  public final double kVLinear = 1;
  public final double kALinear = 1;

  public final double kSAngular = 1;
  public final double kVAngular = 1;
  public final double kAAngular = 1;
}
