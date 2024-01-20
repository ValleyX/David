// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkFlex m_LeftClimber;
  private final CANSparkFlex m_RightClimber;
  private final DigitalInput m_climberDownLimitSwitch;
 // private final CANCoderJNI m_climberAngle;
  private final CANSparkFlex m_PivotCLimbMotor;

  /** Creates a new Climber. */
  public ClimberSubsystem() {
    m_LeftClimber = new CANSparkFlex(CanIDs.kCAN_ClimberMotorLeft, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_RightClimber = new CANSparkFlex(CanIDs.kCAN_ClimberMotorLeft, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_RightClimber.follow(m_LeftClimber); //makes the right motor follow what ever 
    m_PivotCLimbMotor = new CANSparkFlex(CanIDs.kCAN_ClimberPivotMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_climberDownLimitSwitch = new DigitalInput(DigitalInputIDs.kDIG_ClimberDownLimitSwitch);
   // m_climberAngle = new CANCoderJNI();
   // m_climberAngle.ConfigSetParameter();
   
   

  }

  public void ClimbersExtend() {
    m_LeftClimber.set(ClimberConstants.kClimberUpSpeed);
    
  }

  public void ClimbersRetract() {
     m_LeftClimber.set(ClimberConstants.kClimberDownSpeed);
  }

  

  public void ClimberGetAngle(){
    m_PivotCLimbMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle).getPosition(); //TODO check if works 
  }
  public boolean IsClimberDown(){
      return m_climberDownLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // TODO turn climbers off when they reach their given encoder points 
    
    
  }
}
