// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonSRX m_floorIntakeMotor;
  private final DigitalInput m_floorBeans;
  private PWM m_TowerBlinkin;
  private PWM m_BaseBlinkin;

  public IntakeSubsystem() {

    m_floorIntakeMotor = new TalonSRX(RevCanIDs.kCAN_FloorIntakeMotor);
    m_floorBeans = new DigitalInput(DigitalInputIDs.kDIG_FloorBeamBreak);
    SmartDashboard.putBoolean("floor Beam Break", m_floorBeans.get());
    m_floorIntakeMotor.setInverted(true);

    m_TowerBlinkin = new PWM(PWMIDs.kPWM_TowerBlinkin);
    m_BaseBlinkin = new PWM(PWMIDs.kPWM_BaseBlinkin);
  }

  /** Turns on the intake motor to intake in */
  public void in() {

    m_floorIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kInSpeed);
  }

  /** Turns on the intake motor to extake out */
  public void out() {

    m_floorIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kOutSpeed);
  }

  /** Turns off intake motor */
  public void stop() {

    m_floorIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kStop);
  }

  public boolean getFloorBeans() {
    return m_floorBeans.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("floor Beam Break", m_floorBeans.get());
  }
}
