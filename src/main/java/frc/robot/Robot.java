// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.FFDrive;
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;
// import frc.robot.subsystems.Singulator;
// import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.Intake;
import frc.robot.util.ShuffleboardManager;
import frc.robot.util.Vision;
import lib.PathLoader;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoCommand;
  public static Drivetrain drive = new Drivetrain();
  // public static Singulator singulator = new Singulator();
  // public static Intake intake = new Intake();
  public static ShuffleboardManager shuffleboard = new ShuffleboardManager();
  // public static Vision vision = new Vision();
  // public static PIDController controller = new PIDController(1, 0, 0);
  
  // Array of april tags. The index of the april tag in the array is equal to its id, and aprilTags[0] is null.
  public final static Pose3d[] aprilTags = new Pose3d[9];

  // 2D arrays of nodes. blueNodes[3][1] will return the top row cone node on the far left side (from the perspective of the driver)
  public final static Node[][] blueNodes = new Node[4][];
  public final static Node[][] redNodes = new Node[4][];

  // Where the robot will score.
  public Node selectedNode = null;

  // Possible teams
  public static enum Teams {BLUE, RED};
  public static Teams team;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    PathLoader.loadPathGroups();
    
    //drive.setDefaultCommand(new ArcadeDrive());
    drive.setDefaultCommand(new FFDrive());
   //drive.setDefaultCommand(new SenseAprilTagAtVelocity(drive, shuffleboard, controller));

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    Driver.configureControls();
    Operator.configureControls();
    
    shuffleboard.setup();
    Vision.setup();

    // Puts April tags in array
    for(int i = 1; i <= 8; i++){
      aprilTags[i]=Vision.getTagPose(i);
    }

    // Puts nodes in arrays
    for(int i = 1; i <= 3; i++){
      blueNodes[i] = new Node[10];
      redNodes[i] = new Node[10];
      for(int j = 1; j <= 9; j++){
        blueNodes[i][j] = new Node(Teams.BLUE, i, j);
        redNodes[i][j] = new Node(Teams.RED, i, j);
      }
    }

    // Sets robot pose to 1 meter in front of april tag 2
    drive.resetPose(aprilTags[2].getX()-1, aprilTags[2].getY(), 0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    m_autoCommand = getAutonomousCommand();
    team = getTeam();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link Robot} class.
   */
  @Override
  public void autonomousInit() {
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shuffleboard.getAutonomousCommand();
  }
  public Teams getTeam() {
    return shuffleboard.getTeam();
  }
}
