package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.Singulate;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  public static void configureControls() {
    operator.get(Button.A).whenPressed(new Singulate(Robot.m_singulator));
  }

}
