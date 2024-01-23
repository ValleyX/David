// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.sensors.CANCoderJNI;
//import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

//This class holds the 

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkFlex m_ShooterMotor; //Used to expell ring to score, humaning loading
  private final CANSparkFlex m_ShooterPivotMotor; //Used to aim shooter
  private final CANSparkFlex m_ElevatorMotor; // vertical lift of shooting mechanism
  
  private final TalonSRX m_ShooterIntakeMotor; //Used to intake from floor intake motors to feed shooter motors

  private final DigitalInput m_ShooterBeams;
//  private final DigitalInput m_PivotDown;
//  private final DigitalInput m_ElevatorUp;
//  private final DigitalInput m_ElevatorDown;

  private final RelativeEncoder m_ShooterEncoder;
  private final RelativeEncoder m_ElevatorEncoder;
  private final RelativeEncoder m_PivotEncoder;
  
  private SparkPIDController m_PivotPIDController;
  private SparkPIDController m_ElevatorPIDController;

  private SparkLimitSwitch m_PivotIntakeLimitSwitch;
  private SparkLimitSwitch m_ElevatorUpperLimit;
  private SparkLimitSwitch m_ElevatorLowerLimit;

 // public SparkAbsoluteEncoder getAbsoluteEncoder​(SparkAbsoluteEncoder.Type encoderType);

  public ShooterSubsystem() {

    //Declaring motors
    m_ShooterMotor = new CANSparkFlex(CanIDs.kCAN_ShooterMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_ShooterPivotMotor = new CANSparkFlex(CanIDs.kCAN_PivotMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless); //have PID loop
    m_ElevatorMotor = new CANSparkFlex(CanIDs.kCAN_ElevatorMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless); //
    m_ShooterIntakeMotor = new TalonSRX(CanIDs.kCAN_ShooterIntakeMotor);
  
    //These lines set the two motors to their default settings
    m_ShooterPivotMotor.restoreFactoryDefaults();
    m_ElevatorMotor.restoreFactoryDefaults(); 
    m_ElevatorMotor.restoreFactoryDefaults();
    m_ShooterIntakeMotor.configFactoryDefault();

    m_ShooterBeams = new DigitalInput(DigitalInputIDs.kDIG_ShootBeamBreak); //shooter beam break
   // m_PivotDown = new DigitalInput(DigitalInputIDs.kDIG_PivotDownLimitSwitch);
   // m_ElevatorUp = new DigitalInput(DigitalInputIDs.kDIG_ElevatorUpLimitSwitch); //These are for the digital inputs for limits may use later
  //  m_ElevatorDown = new DigitalInput(DigitalInputIDs.kDIG_ElevatorDownLimitSwitch);

    m_ElevatorEncoder = m_ElevatorMotor.getEncoder(); //TODO have to check later
    m_ElevatorPIDController = m_ElevatorMotor.getPIDController();//setting up PID controller from the motor
    m_ElevatorPIDController.setFeedbackDevice(m_ElevatorEncoder);//setting the encoder to feed back into the motor to make the motor beable to go to specfic cordinates



    m_ShooterEncoder = m_ShooterMotor.getEncoder(); //TODO have to check later NEED TO ADD VELOCITY ff PID

    m_PivotEncoder = m_ShooterPivotMotor.getEncoder();
    m_PivotPIDController = m_ShooterPivotMotor.getPIDController(); //setting up PID controller from the motor
    m_PivotPIDController.setFeedbackDevice(m_ShooterEncoder); //setting the encoder to feed back into the motor to make the motor beable to go to specfic cordinates

    //TODO Theses could be normally closed or open, Ask matt, normally closed means it runs voltage   
    // these are to tell the motor if you have hit the max or min(boolean) and the motor cuts off if so
    m_ElevatorUpperLimit = m_ElevatorMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen); 
    m_ElevatorLowerLimit = m_ElevatorMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);  
    m_PivotIntakeLimitSwitch = m_ShooterPivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen); 
    
    
    //seting PID coeficients for pivot motor
    m_PivotPIDController.setP(ShooterConstants.kP);
    m_PivotPIDController.setI(ShooterConstants.kI);
    m_PivotPIDController.setD(ShooterConstants.kD);
    m_PivotPIDController.setIZone(ShooterConstants.kIz);
    m_PivotPIDController.setFF(ShooterConstants.kFF);
    m_PivotPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    //Display PID cofficients for shooter pivot motor on Smartboard
    SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
    SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
    SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ShooterConstants.kFF);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Degrees of Pivot: ", m_PivotEncoder.getPosition()/360);

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
    SmartDashboard.putNumber("Elevator Position",m_ElevatorEncoder.getPosition());

  }

  public void MovePivotTo(double positionDeg){ // Move Pivot motor to specific location by PID loop
   // m_PivotPIDController
   //TODO take degrees to rotations
   
   m_PivotPIDController.setReference((positionDeg*360),ControlType.kPosition); //TODO check if works and put rotations as input
  }

  public void MoveElevator(double speed){//moves elevator with a given speed
    m_ElevatorMotor.set(speed);
  }

  //function to move elevator to specific height
  public void MoveElevatorTo(double heightInches){ 
       m_ElevatorPIDController.setReference(heightInches,ControlType.kPosition); // TODO have to check if works, and conversion of units may need to happen 
  }

  public double elevatorPos(){
      return m_ElevatorEncoder.getPosition();
  }

  public void MovePivot(double speed){//moves pivot with a given speed
    m_ShooterPivotMotor.set(speed);
  }

  public void MoveShooter(double speed){//moves shooter with a given speed
    m_ShooterMotor.set(speed);
  }

  public double GetPivotPosition(){
    return m_ShooterEncoder.getPosition(); //TODO; check if works
  }

  public double GetElevatorPosition(){
    return m_ElevatorEncoder.getPosition(); //TODO; fill this in with a value
  }

  public double GetRPM(){
    return m_ShooterEncoder.getVelocity();
  }

  public double GetPos(){
    return m_ShooterEncoder.getPosition();
  }


  public void IntakeIn(){//turns on shooter intake on to intake toward shooter
    m_ShooterIntakeMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.inTake );
  }

  public void ExtakeOut(){//turns on shooter intake on to extake away from shooter
    m_ShooterIntakeMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.OutTake );
  }

  public void IntakeStop(){//stops intake motor
    m_ShooterIntakeMotor.set(TalonSRXControlMode.PercentOutput,0);

  }


  public boolean IsShooterAligned(){//checks if pivot limit switch is active from the motors
   return m_PivotIntakeLimitSwitch.isPressed();
  }

  public boolean IsNoteRecieved(){//checks if shooter beambreak is broken to see if we have a note
    return m_ShooterBeams.get();
  }

  public boolean IsElevatorUp(){//checks if elevator is all the way up from the motors
    return m_ElevatorUpperLimit.isPressed();
  }

  public boolean IsElevatorDown(){//checks if elevator is all the way Down from the motors
    return m_ElevatorLowerLimit.isPressed();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

      //Display PID cofficients for shooter pivot motor on Smartboard 
    double shooterPivotP = SmartDashboard.getNumber("shooterPivotP Gain", ShooterConstants.kP);
    double shooterPivotI = SmartDashboard.getNumber("shooterPivotI Gain", ShooterConstants.kI);
    double shooterPivotD = SmartDashboard.getNumber("shooterPivotD Gain", ShooterConstants.kD);
    double shooterPivotIzone = SmartDashboard.getNumber("shooterPivot I Zone", ShooterConstants.kIz);
    double shooterPivotff = SmartDashboard.getNumber("shooter pivot Feed Forward", ShooterConstants.kFF);
    double shooterPivotMax = SmartDashboard.getNumber("shooter pivot Max Output", ShooterConstants.kMaxOutput);
    double shooterPivotMin = SmartDashboard.getNumber("shooter pivot Min Output", ShooterConstants.kMinOutput);
    double shooterPivotRotations = SmartDashboard.getNumber("shooter pivot Set Rotations", 0);

    if((shooterPivotP != ShooterConstants.kP)){ m_PivotPIDController.setP(shooterPivotP); shooterPivotP = ShooterConstants.kP;} //making the shooterpivot PID to match the PIDs on the Dashboard
    if((shooterPivotI != ShooterConstants.kI)){m_PivotPIDController.setI(shooterPivotI); shooterPivotI = ShooterConstants.kI;}
    if((shooterPivotD != ShooterConstants.kD)){m_PivotPIDController.setD(shooterPivotD); shooterPivotD = ShooterConstants.kD;}
    if((shooterPivotIzone != ShooterConstants.kIz)){m_PivotPIDController.setIZone(shooterPivotIzone); shooterPivotIzone = ShooterConstants.kIz;}
    if((shooterPivotff != ShooterConstants.kFF)){m_PivotPIDController.setFF(shooterPivotff); shooterPivotff = ShooterConstants.kFF;}
    if((shooterPivotMax != ShooterConstants.kMaxOutput)  || (shooterPivotMin != ShooterConstants.kMinOutput)){
      m_PivotPIDController.setOutputRange(shooterPivotMin,shooterPivotMax);
      shooterPivotMax = ShooterConstants.kMaxOutput; shooterPivotMin = ShooterConstants.kMinOutput;
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

    if((elavatorP != ElevatorConstants.kP)){ m_ElevatorPIDController.setP(elavatorP); elavatorP = ElevatorConstants.kP;} //making the shooterpivot PID to match the PIDs on the Dashboard
    if((elavatorI != ElevatorConstants.kI)){m_ElevatorPIDController.setI(elavatorI); elavatorI = ElevatorConstants.kI;}
    if((elavatorD != ElevatorConstants.kD)){m_ElevatorPIDController.setD(elavatorD); elavatorD = ElevatorConstants.kD;}
    if((elavatorIzone != ElevatorConstants.kIz)){m_ElevatorPIDController.setIZone(elavatorIzone); elavatorIzone = ElevatorConstants.kIz;}
    if((elavatorff != ElevatorConstants.kFF)){m_ElevatorPIDController.setFF(elavatorff); elavatorff = ElevatorConstants.kFF;}
    if((elavatorMax != ElevatorConstants.kMaxOutput)  || (elavatorMin != ElevatorConstants.kMinOutput)){
      m_PivotPIDController.setOutputRange(elavatorMin,elavatorMax);
      shooterPivotMax = ShooterConstants.kMaxOutput; elavatorMin = ShooterConstants.kMinOutput;
    }
    m_PivotPIDController.setReference(elavatorRotations,ControlType.kPosition); // This makes the PID loop go


  }

}
