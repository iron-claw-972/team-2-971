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


public class Outtake extends SubsystemBase {

  private WPI_TalonSRX m_outtakeMotor;

  public Outtake() {
    this(Motors.createTalonSRX(Constants.outtake.kOuttakeMotor, NeutralMode.Brake));
  }

  public Outtake(WPI_TalonSRX outtakeMotor) {
    m_outtakeMotor = outtakeMotor;
  }

  public void outtake() {
    m_outtakeMotor.set(Constants.outtake.kOuttakeSpeed);
  }

  public void stop() {
    m_outtakeMotor.set(0);
  }
}
