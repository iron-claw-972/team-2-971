package frc.robot.controls;

import frc.robot.constants.Constants;
import lib.controllers.GameController;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  public static void configureControls() {
    // operator.get(Button.A).whenPressed(new Singulate(Robot.m_singulator));
    // operator.get(Button.B).whenPressed(new IntakeBall(Robot.m_intake));
  }

}
