// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.sensors.CANCoderJNI;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkFlex m_ShooterMotor;
  private final CANSparkFlex m_PivotMotor;
  private final CANSparkFlex m_ElevatorMotor;
  
  private final TalonSRX m_ShooterIntakeMotor; 

  private final DigitalInput m_ShooterBeans;
  private final DigitalInput m_PivotDown;
  private final DigitalInput m_ElevatorUp;
  private final DigitalInput m_ElevatorDown;

  private final RelativeEncoder m_PivotEncoder;
  private final RelativeEncoder m_ElevatorEncoder;
  
  private SparkPIDController m_PivotPIDController;
  private SparkPIDController m_ElevatorPIDController;
 // public SparkAbsoluteEncoder getAbsoluteEncoder​(SparkAbsoluteEncoder.Type encoderType);

  public ShooterSubsystem() {

    m_ShooterMotor = new CANSparkFlex(CanIDs.kCAN_ShooterMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_PivotMotor = new CANSparkFlex(CanIDs.kCAN_PivotMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless); //have PID loop
    m_ElevatorMotor = new CANSparkFlex(CanIDs.kCAN_ElevatorMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless); //needs PID loop?
  

    m_ShooterIntakeMotor = new TalonSRX(CanIDs.kCAN_ShooterIntakeMotor);

    m_ShooterBeans = new DigitalInput(DigitalInputIDs.kDIG_ShootBeanBreak); //shooter bean break
    m_PivotDown = new DigitalInput(DigitalInputIDs.kDIG_PivotDownLimitSwitch);
    m_ElevatorUp = new DigitalInput(DigitalInputIDs.kDIG_ElevatorUpLimitSwitch);
    m_ElevatorDown = new DigitalInput(DigitalInputIDs.kDIG_ElevatorDownLimitSwitch);

    m_ElevatorEncoder = m_ElevatorMotor.getEncoder(); //TODO have to check later
    m_ElevatorPIDController = m_ElevatorMotor.getPIDController();//setting up PID controller from the motor
    m_ElevatorPIDController.setFeedbackDevice(m_ElevatorEncoder);//setting the encoder to feed back into the motor to make the motor beable to go to specfic cordinates



    m_PivotEncoder = m_PivotMotor.getEncoder(); //TODO have to check later
    m_PivotPIDController = m_PivotMotor.getPIDController(); //setting up PID controller from the motor
    m_PivotPIDController.setFeedbackDevice(m_PivotEncoder); //setting the encoder to feed back into the motor to make the motor beable to go to specfic cordinates

    //seting PID coeficients for pivot motor
    m_PivotPIDController.setP(ShooterPivotPID.kP);
    m_PivotPIDController.setI(ShooterPivotPID.kI);
    m_PivotPIDController.setD(ShooterPivotPID.kD);
    m_PivotPIDController.setIZone(ShooterPivotPID.kIz);
    m_PivotPIDController.setFF(ShooterPivotPID.kFF);
    m_PivotPIDController.setOutputRange(ShooterPivotPID.kMinOutput, ShooterPivotPID.kMaxOutput);

    //Display PID cofficients for shooter pivot motor on Smartboard
    SmartDashboard.putNumber("P Gain", ShooterPivotPID.kP);
    SmartDashboard.putNumber("I Gain", ShooterPivotPID.kI);
    SmartDashboard.putNumber("D Gain", ShooterPivotPID.kD);
    SmartDashboard.putNumber("I Zone", ShooterPivotPID.kIz);
    SmartDashboard.putNumber("Feed Forward", ShooterPivotPID.kFF);
    SmartDashboard.putNumber("Max Output", ShooterPivotPID.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterPivotPID.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    //setting PID coefficients for Elevator motor
    m_PivotPIDController.setP(ElevatorConstants.kP);
    m_PivotPIDController.setI(ElevatorConstants.kI);
    m_PivotPIDController.setD(ElevatorConstants.kD);
    m_PivotPIDController.setIZone(ElevatorConstants.kIz);
    m_PivotPIDController.setFF(ElevatorConstants.kFF);
    m_PivotPIDController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);

    //Display PID cofficients for Elevator on Smartboard
    SmartDashboard.putNumber("P Gain", ElevatorConstants.kP);
    SmartDashboard.putNumber("I Gain", ElevatorConstants.kI);
    SmartDashboard.putNumber("D Gain", ElevatorConstants.kD);
    SmartDashboard.putNumber("I Zone", ElevatorConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ElevatorConstants.kFF);
    SmartDashboard.putNumber("Max Output", ElevatorConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ElevatorConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);




  }

  public void MovePivotTo(double positionInches){ // Move Pivot motor to specific location by PID loop
   // m_PivotPIDController
   m_PivotPIDController.setReference(positionInches,ControlType.kPosition); //TODO check if works

  }

  public void MoveElevator(double speed){//moves elevator with a given speed
    m_ElevatorMotor.set(speed);
  }

  public void MoveElevatorTo(double heightInches){ //function to move elevator to specific height
       m_ElevatorPIDController.setReference(heightInches,ControlType.kPosition);
  }

  public void MovePivot(double speed){//moves pivot with a given speed
    m_PivotMotor.set(speed);
    //m_PivotMotor.
  }

  public void MoveShooter(double speed){//moves shooter with a given speed
    m_ShooterMotor.set(speed);
  }

  public double GetPivotPosition(){
    return m_PivotEncoder.getPosition(); //TODO; check if works
  }

  public double GetElevatorPosition(){
    return m_ElevatorEncoder.getPosition(); //TODO; fill this in with a value
  }

  public double GetRPM(){
    return 0; //TODO; fill this in with a value
  }


  public void IntakeIn(){//turns on shooter intake on to intake toward shooter

  }

  public void ExtakeOut(){//turns on shooter intake on to extake away from shooter

  }

  public void IntakeStop(){//stops intake motor

  }


  public boolean IsShooterAligned(){//checks if pivot limit switch is active
   return m_PivotDown.get(); 
  }

  public boolean IsNoteRecieved(){//checks if shooter beambreak is broken to see if we have a note
    return m_ShooterBeans.get();
  }

  public boolean IsElevatorUp(){//checks if elevator is all the way up
    return m_ElevatorUp.get();
  }

  public boolean IsElevatorDown(){//checks if elevator is all the way Down
    return m_ElevatorDown.get();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

      //Display PID cofficients for shooter pivot motor on Smartboard 
    double shooterPivotP = SmartDashboard.getNumber("shooterPivotP Gain", ShooterPivotPID.kP);
    double shooterPivotI = SmartDashboard.getNumber("shooterPivotI Gain", ShooterPivotPID.kI);
    double shooterPivotD = SmartDashboard.getNumber("shooterPivotD Gain", ShooterPivotPID.kD);
    double shooterPivotIzone = SmartDashboard.getNumber("shooterPivot I Zone", ShooterPivotPID.kIz);
    double shooterPivotff = SmartDashboard.getNumber("shooter pivot Feed Forward", ShooterPivotPID.kFF);
    double shooterPivotMax = SmartDashboard.getNumber("shooter pivot Max Output", ShooterPivotPID.kMaxOutput);
    double shooterPivotMin = SmartDashboard.getNumber("shooter pivot Min Output", ShooterPivotPID.kMinOutput);
    double shooterPivotRotations = SmartDashboard.getNumber("shooter pivot Set Rotations", 0);

    if((shooterPivotP != ShooterPivotPID.kP)){ m_PivotPIDController.setP(shooterPivotP); shooterPivotP = ShooterPivotPID.kP;} 
    if((shooterPivotI != ShooterPivotPID.kI)){m_PivotPIDController.setI(shooterPivotI); shooterPivotI = ShooterPivotPID.kI;}
    if((shooterPivotD != ShooterPivotPID.kD)){m_PivotPIDController.setD(shooterPivotD); shooterPivotD = ShooterPivotPID.kD;}
    if((shooterPivotIzone != ShooterPivotPID.kIz)){m_PivotPIDController.setIZone(shooterPivotIzone); shooterPivotIzone = ShooterPivotPID.kIz;}
    if((shooterPivotff != ShooterPivotPID.kFF)){m_PivotPIDController.setFF(shooterPivotff); shooterPivotff = ShooterPivotPID.kFF;}
    if((shooterPivotMax != ShooterPivotPID.kMaxOutput)  || (shooterPivotMin != ShooterPivotPID.kMinOutput)){
      m_PivotPIDController.setOutputRange(shooterPivotMin,shooterPivotMax);
      shooterPivotMax = ShooterPivotPID.kMaxOutput; shooterPivotMin = ShooterPivotPID.kMinOutput;
    }
    m_PivotPIDController.setReference(shooterPivotRotations,ControlType.kPosition); // This makes the PID loop go

    //Display PID cofficients for elavator motor on Smartboard 
    double elavatorP = SmartDashboard.getNumber("elavator P Gain", ElevatorConstants.kP);
    double elavatorI = SmartDashboard.getNumber("elavator I Gain", ElevatorConstants.kI);
    double elavatorD = SmartDashboard.getNumber("elavator D Gain", ElevatorConstants.kD);
    double elavatorIzone = SmartDashboard.getNumber("elavator I Zone", ElevatorConstants.kIz);
    double elavatorff = SmartDashboard.getNumber("elavator Feed Forward", ElevatorConstants.kFF);
    double elavatorMax = SmartDashboard.getNumber("shooter pivot Max Output", ElevatorConstants.kMaxOutput);
    double elavatorMin = SmartDashboard.getNumber("shooter pivot Min Output", ElevatorConstants.kMinOutput);
    double elavatorRotations = SmartDashboard.getNumber("shooter pivot Set Rotations", 0);

    if((elavatorP != ElevatorConstants.kP)){ m_ElevatorPIDController.setP(elavatorP); elavatorP = ElevatorConstants.kP;} 
    if((elavatorI != ElevatorConstants.kI)){m_ElevatorPIDController.setI(elavatorI); elavatorI = ElevatorConstants.kI;}
    if((elavatorD != ElevatorConstants.kD)){m_ElevatorPIDController.setD(elavatorD); elavatorD = ElevatorConstants.kD;}
    if((elavatorIzone != ElevatorConstants.kIz)){m_ElevatorPIDController.setIZone(elavatorIzone); elavatorIzone = ElevatorConstants.kIz;}
    if((elavatorff != ElevatorConstants.kFF)){m_ElevatorPIDController.setFF(elavatorff); elavatorff = ElevatorConstants.kFF;}
    if((elavatorMax != ElevatorConstants.kMaxOutput)  || (elavatorMin != ElevatorConstants.kMinOutput)){
      m_PivotPIDController.setOutputRange(elavatorMin,elavatorMax);
      shooterPivotMax = ShooterPivotPID.kMaxOutput; elavatorMin = ShooterPivotPID.kMinOutput;
    }
    m_PivotPIDController.setReference(elavatorRotations,ControlType.kPosition); // This makes the PID loop go


  }

}
