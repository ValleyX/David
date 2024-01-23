// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;


public class IntakeSubsystem extends SubsystemBase {

  private final TalonSRX m_floorIntakeMotor; 
  private final DigitalInput m_floorBeans;

  public IntakeSubsystem() {

    m_floorIntakeMotor = new TalonSRX(CanIDs.kCAN_FloorIntakeMotor);
    m_floorBeans = new DigitalInput(DigitalInputIDs.kDIG_FloorBeamBreak);
  }

  public void IntakeIn() {

    m_floorIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kIntakeInSpeed );

  }

   public void ExtakeOut() {

    m_floorIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kExtakeOutSpeed );

  }

   public void IntakeStop() {

    m_floorIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kIntakeStop );

  }

  @Override
  public void periodic() {

  }
}
