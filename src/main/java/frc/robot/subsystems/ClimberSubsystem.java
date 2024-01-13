// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkFlex m_LeftClimber;
  private final CANSparkFlex m_RightClimber;

  /** Creates a new Climber. */
  public ClimberSubsystem() {
    m_LeftClimber = new CANSparkFlex(CanIDs.kCAN_ClimberMotorLeft, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_RightClimber = new CANSparkFlex(CanIDs.kCAN_ClimberMotorLeft, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  }

  public void ClimbersExtend() {
    m_LeftClimber.set(ClimberConstants.kClimberUpSpeed);
    m_RightClimber.set(ClimberConstants.kClimberUpSpeed);
  }

  public void ClimbersRetract() {
     m_LeftClimber.set(ClimberConstants.kClimberDownSpeed);
    m_RightClimber.set(ClimberConstants.kClimberDownSpeed);
  }

  @Override
  public void periodic() {
    // TODO turn climbers off when they reach their given encoder points 
    
  }
}
