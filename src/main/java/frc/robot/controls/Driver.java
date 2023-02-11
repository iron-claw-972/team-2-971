package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Align;
import frc.robot.commands.TestCameraPose;
import frc.robot.commands.TestVision;
import frc.robot.commands.TestVision2;
import frc.robot.constants.Constants;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(Constants.drive.kDriveSlewRate);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(Constants.drive.kDriveSlewRate);

  public static void configureControls() {
    // driver.get(Button.Y).whenPressed(new AlignToPlant(true));
    // driver.get(Button.A).whenPressed(new AlignToPlant(false));
    driver.get(Button.X).onTrue(new InstantCommand(()->Robot.drive.printPose()));
    driver.get(Button.B).onTrue(new InstantCommand(()->Vision.printEstimate()));
    driver.get(Button.RB).onTrue(new TestVision(-0.1));
    driver.get(Button.LB).onTrue(new TestVision(0.1));
    driver.get(Button.A).onTrue(new TestCameraPose(0.3));
    driver.get(Button.Y).onTrue(new SequentialCommandGroup(new InstantCommand(()->Vision.printEstimate()), new InstantCommand(()->Robot.drive.printPose())));
    driver.get(driver.RIGHT_TRIGGER_BUTTON).onTrue(new TestVision2(-0.1, 3));
    driver.get(driver.LEFT_TRIGGER_BUTTON).onTrue(new TestVision2(0.1, 3));
    driver.get(DPad.LEFT).onTrue(new Align(0, Robot.drive));
    driver.get(DPad.DOWN).onTrue(new Align(Math.PI/2, Robot.drive));
    driver.get(DPad.RIGHT).onTrue(new Align(Math.PI, Robot.drive));
    driver.get(DPad.UP).onTrue(new Align(-Math.PI/2, Robot.drive));
  }

  public static void configureTestControls() {
  }

  public static double getThrottleValue() {
    // put any processes in any order of the driver's choosing
    // Controllers y-axes are natively up-negative, down-positive
    return slewThrottle.calculate(MathUtil.applyDeadband(getRawThrottleValue(), Constants.drive.kDeadband));
  }

  public static double getTurnValue() {
    // right is positive; left is negative
    return slewTurn.calculate(MathUtil.applyDeadband(getRawTurnValue(), Constants.drive.kDeadband));
  }

  public static double getRawThrottleValue(){
    return driver.get(Axis.LEFT_Y);
  }
  public static double getRawTurnValue(){
    return -driver.get(Axis.RIGHT_X);
  }
}
