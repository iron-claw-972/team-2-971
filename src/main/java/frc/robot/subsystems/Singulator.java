/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Motors;


public class Singulator extends SubsystemBase {

  public CANSparkMax m_motor;

  public Singulator() {
    this(Motors.createSparkMAX(0, MotorType.kBrushless, IdleMode.kCoast));
  }

  public Singulator(CANSparkMax motor) {
    m_motor = motor;
  }

  public void set(double power) {
    m_motor.set(power);
  }

  public void stop() {
    m_motor.set(0);
  }
 
}
