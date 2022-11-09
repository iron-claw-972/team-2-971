package frc.robot.controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);
  private static SlewRateLimiter slewThrottle = new SlewRateLimiter(Constants.drive.kSlewRate);
  private static SlewRateLimiter slewTurn = new SlewRateLimiter(Constants.drive.kSlewRate);



  public static void configureControls() {
    driver.get(Button.A).whenPressed(new DoNothing());
  }

  public static double getThrottleValue() {
    // put any processes in any order of the driver's choosing
    // Controllers y-axes are natively up-negative, down-positive
      //return Functions.deadband(0.05, driver.get(Axis.LEFT_Y));
      return -slewThrottle.calculate(Functions.deadband(Constants.oi.kDeadband, getRawThrottleValue()));

  }

  public static double getTurnValue() {
    // right is positive; left is negative
    //return Functions.deadband(0.05, driver.get(Axis.RIGHT_X));
    return -slewTurn.calculate(Functions.deadband(Constants.oi.kDeadband, getRawTurnValue()));
  }

  public static double getRawThrottleValue(){
    return driver.get(Axis.LEFT_Y);
  }
  public static double getRawTurnValue(){
    return driver.get(Axis.RIGHT_X);
  }


}
