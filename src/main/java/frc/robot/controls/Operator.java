package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.DPad;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  public static void configureControls() {
    // operator.get(Button.A).whenPressed(new Singulate(Robot.m_singulator));
    // operator.get(Button.B).whenPressed(new IntakeBall(Robot.m_intake));
    operator.get(DPad.LEFT).onTrue(new InstantCommand(()->Robot.DPadPress(DPad.LEFT)));
    operator.get(DPad.UP).onTrue(new InstantCommand(()->Robot.DPadPress(DPad.UP)));
    operator.get(DPad.RIGHT).onTrue(new InstantCommand(()->Robot.DPadPress(DPad.RIGHT)));
    operator.get(DPad.DOWN).onTrue(new InstantCommand(()->Robot.DPadPress(DPad.DOWN)));
  }
  }

}
