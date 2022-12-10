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


public class Intake extends SubsystemBase {

  private CANSparkMax m_singulatorMotor;
  private WPI_TalonSRX m_intakeMotor;

  public Intake() {
    this(
      Motors.createSparkMAX(Constants.intake.kIntakeMotor, MotorType.kBrushless, IdleMode.kCoast),
      Motors.createTalonSRX(Constants.intake.kSingulatorMotor, NeutralMode.Brake)
    );
  }

  public Intake(CANSparkMax singulatorMotor, WPI_TalonSRX intakeMotor) {
    m_singulatorMotor = singulatorMotor;
    m_intakeMotor = intakeMotor;
  }

  public void intake() {
    m_singulatorMotor.set(Constants.intake.kSingulatorSpeed);
    m_intakeMotor.set(Constants.intake.kIntakeSpeed);
  }
  
  public void stop() {
    m_singulatorMotor.set(0);
    m_intakeMotor.set(0);
  }
 
}
