/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import lib.Motors;


public class Feeder extends SubsystemBase {

  private CANSparkMax m_feedMotor;

  public Feeder() {
    this(Motors.createSparkMAX(Constants.feeder.kFeedMotor, MotorType.kBrushless, IdleMode.kBrake));
  }

  public Feeder(CANSparkMax feedMotor) {
    m_feedMotor = feedMotor;
  }

  public void feed(boolean left) {
    m_feedMotor.set((left ? -1 : 1) * Constants.feeder.kFeedSpeed);
  }

  public void stop() {
    m_feedMotor.set(0);
  }
}
