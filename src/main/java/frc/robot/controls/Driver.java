package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(Constants.drive.kDriveSlewRate);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(Constants.drive.kDriveSlewRate);

  public static void configureControls() {
    // driver.get(Button.Y).whenPressed(new AlignToPlant(true));
    // driver.get(Button.A).whenPressed(new AlignToPlant(false));
    driver.get(Button.X).onTrue(new InstantCommand(()->Robot.drive.printPose()));
    driver.get(Button.Y).toggleOnTrue(new RunCommand(()->Robot.drive.printVisionPose()));
    driver.get(Button.B).onTrue(new InstantCommand(()->Vision.printEstimate()));
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
