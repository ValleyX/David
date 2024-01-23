// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkFlex m_LeftClimber;
  private final CANSparkFlex m_RightClimber;

 // private final CANCoderJNI m_climberAngle;

  private final RelativeEncoder m_ClimberEncoder;
  
  private SparkPIDController m_ClimberPIDController;
  private SparkLimitSwitch climberDown;

  /** Creates a new Climber. */
  public ClimberSubsystem() {
    m_LeftClimber = new CANSparkFlex(CanIDs.kCAN_ClimberMotorLeft, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_RightClimber = new CANSparkFlex(CanIDs.kCAN_ClimberMotorLeft, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_LeftClimber.restoreFactoryDefaults(); //
    m_RightClimber.restoreFactoryDefaults();
    m_RightClimber.follow(m_LeftClimber); //makes the right motor follow what ever 
   

  
  

    //TODO Theses could be normally closed or open  
    // these are to tell the motor if you have hit the max or min(boolean) and the motor cuts off if so it doesn't break 
    climberDown = m_LeftClimber.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);


    m_ClimberEncoder = m_LeftClimber.getEncoder(); //TODO have to check later
    m_ClimberPIDController = m_LeftClimber.getPIDController();//setting up PID controller from the motor
    m_ClimberPIDController.setFeedbackDevice(m_ClimberEncoder);//setting the encoder to feed back into the motor to make the motor beable to go to specfic cordinates

    //seting PID coeficients for climb motor
    m_ClimberPIDController.setP(ClimberConstants.kP);
    m_ClimberPIDController.setI(ClimberConstants.kI);
    m_ClimberPIDController.setD(ClimberConstants.kD);
    m_ClimberPIDController.setIZone(ClimberConstants.kIz);
    m_ClimberPIDController.setFF(ClimberConstants.kFF);
    m_ClimberPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    //Display PID cofficients for shooter pivot motor on Smartboard
    SmartDashboard.putNumber("P Gain", ClimberConstants.kP);
    SmartDashboard.putNumber("I Gain", ClimberConstants.kI);
    SmartDashboard.putNumber("D Gain", ClimberConstants.kD);
    SmartDashboard.putNumber("I Zone", ClimberConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ClimberConstants.kFF);
    SmartDashboard.putNumber("Max Output", ClimberConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ClimberConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  public void ClimbersExtend() {
    m_LeftClimber.set(ClimberConstants.kClimberUpSpeed);
    
  }

  public void ClimbersRetract() {
     m_LeftClimber.set(ClimberConstants.kClimberDownSpeed);
  }

  
  /**
   * gets the absolute encoder position from the pivot climb motor
   * @return a double in motor rotations
   */
  public double ClimberGetAngle(){
    return m_LeftClimber.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle).getPosition(); //TODO check if works 
  }
  public boolean IsClimberDown(){ //checks limit switch to see if climber is down
      return climberDown.isPressed();
  }



  @Override
  public void periodic() {
    // TODO turn climbers off when they reach their given encoder points 
  
    //getting PID values from smartDashBoard
    double climberPivotP = SmartDashboard.getNumber("climberPivotP Gain", ClimberConstants.kP);
    double climberPivotI = SmartDashboard.getNumber("climberPivotI Gain", ClimberConstants.kI);
    double climberPivotD = SmartDashboard.getNumber("climberPivotD Gain", ClimberConstants.kD);
    double climberPivotIzone = SmartDashboard.getNumber("climberPivot I Zone", ClimberConstants.kIz);
    double climberPivotff = SmartDashboard.getNumber("climberPivot Feed Forward", ClimberConstants.kFF);
    double climberPivotMax = SmartDashboard.getNumber("climberPivot Max Output", ClimberConstants.kMaxOutput);
    double climberPivotMin = SmartDashboard.getNumber("climberPivot Min Output", ClimberConstants.kMinOutput);
    double climberPivotRotations = SmartDashboard.getNumber("climberPivot Set Rotations", 0);

    //making sure PID loop values match the smartboard values
    if((climberPivotP != ClimberConstants.kP)){ m_ClimberPIDController.setP(climberPivotP); climberPivotP = ClimberConstants.kP;} 
    if((climberPivotI != ClimberConstants.kI)){m_ClimberPIDController.setI(climberPivotI); climberPivotI = ClimberConstants.kI;}
    if((climberPivotD != ClimberConstants.kD)){m_ClimberPIDController.setD(climberPivotD); climberPivotD = ClimberConstants.kD;}
    if((climberPivotIzone != ClimberConstants.kIz)){m_ClimberPIDController.setIZone(climberPivotIzone); climberPivotIzone = ClimberConstants.kIz;}
    if((climberPivotff != ClimberConstants.kFF)){m_ClimberPIDController.setFF(climberPivotff); climberPivotff = ClimberConstants.kFF;}
    if((climberPivotMax != ClimberConstants.kMaxOutput)  || (climberPivotMin != ClimberConstants.kMinOutput)){
      m_ClimberPIDController.setOutputRange(climberPivotMin,climberPivotMax);
      climberPivotMax = ClimberConstants.kMaxOutput; climberPivotMin = ClimberConstants.kMinOutput;
    }

    m_ClimberPIDController.setReference(climberPivotRotations,ControlType.kPosition); // This makes the PID loop go
    
  }
}
