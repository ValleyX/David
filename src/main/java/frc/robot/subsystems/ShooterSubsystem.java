// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoderJNI;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkFlex m_ShooterMotor;
  private final CANSparkFlex m_PivotMotor;
  private final CANSparkFlex m_ElevatorMotor;
  
  private final TalonSRX m_ShooterIntakeMotor; 

  private final DigitalInput m_ShooterBeans;
  private final DigitalInput m_PivotDown;

  private final CANCoderJNI m_PivotEncoder;

  public ShooterSubsystem() {

    m_ShooterMotor = new CANSparkFlex(CanIDs.kCAN_ShooterMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_PivotMotor = new CANSparkFlex(CanIDs.kCAN_PivotMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_ElevatorMotor = new CANSparkFlex(CanIDs.kCAN_ElevatorMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
   
    m_ShooterIntakeMotor = new TalonSRX(CanIDs.kCAN_ShooterIntakeMotor);

    m_ShooterBeans = new DigitalInput(DigitalInputIDs.kDIG_ShootBeanBreak);
    m_PivotDown = new DigitalInput(DigitalInputIDs.kDIG_PivotDownLimitSwitch);

    m_PivotEncoder = new CANCoderJNI();
    
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
