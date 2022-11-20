package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.commands.auto.PathPlannerCommand;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  ShuffleboardTab m_intakeTab = Shuffleboard.getTab("Intake");
  ShuffleboardTab m_SingulatorTab = Shuffleboard.getTab("Singulator");


  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();
  
  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    setupAutoChooser();
    loadAllTabs();
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  public void setupAutoChooser() {
    m_autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
    m_autoCommand.addOption("TestAuto", new PathPlannerCommand("TestAuto", 0)); 
    m_autoCommand.addOption("OneDotAuto", new PathPlannerCommand("OneDotAuto", 0)); 
    m_autoCommand.addOption("TwoDotAuto", new PathPlannerCommand("TwoDotAuto", 0)); 
    m_autoCommand.addOption("ThreeDotAuto", new PathPlannerCommand("ThreeDotAuto", 0)); 
  }

  public void loadAllTabs() {
    // put subsystem shuffleboard things in here
    loadMainTab();
    loadAutoTab();
    loadDriveTab();
    loadIntakeTab();
    loadSingulatorTab();
  }

  public void loadMainTab(){
    m_mainTab.addBoolean("Is Teleop", DriverStation::isTeleop);
    m_mainTab.addNumber("Time Left", DriverStation::getMatchTime);
    
  }

  public void loadAutoTab(){
    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_autoTab.addNumber("X", Robot.drive::getPoseX);
    m_autoTab.addNumber("Y", Robot.drive::getPoseX);
    m_autoTab.addNumber("Rotation", Robot.drive::getPoseRotation);
  }

  public void loadDriveTab(){
    m_mainTab.addNumber("Drivetrain Speed", Robot.drive::getDriveSpeed);
    m_autoTab.addNumber("NavX Position", Robot.drive::getGyroRotation);
  }

  public void loadIntakeTab(){

  }

  public void loadSingulatorTab(){

  }


  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }

}
