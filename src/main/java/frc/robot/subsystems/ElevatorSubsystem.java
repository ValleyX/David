// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.RevCanIDs;
import frc.robot.Constants.ShooterConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final CANSparkFlex m_ElevatorMotor; // vertical lift of shooting mechanism
  private final RelativeEncoder m_ElevatorEncoder;
  private SparkPIDController m_ElevatorPIDController;
  private SparkLimitSwitch m_ElevatorUpperLimit;
  private SparkLimitSwitch m_ElevatorLowerLimit;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_ElevatorMotor =
        new CANSparkFlex(
            RevCanIDs.kCAN_ElevatorMotor,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless); //

    m_ElevatorMotor.restoreFactoryDefaults();

    m_ElevatorMotor.setInverted(true);

    m_ElevatorEncoder = m_ElevatorMotor.getEncoder(); // TODO have to check later
    m_ElevatorPIDController =
        m_ElevatorMotor.getPIDController(); // setting up PID controller from the motor
    m_ElevatorPIDController.setFeedbackDevice(
        m_ElevatorEncoder); // setting the encoder to feed back into the motor to make the motor
    m_ElevatorEncoder.setPosition(0);
    m_ElevatorMotor.setIdleMode(IdleMode.kBrake);

    m_ElevatorUpperLimit =
        m_ElevatorMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    m_ElevatorLowerLimit =
        m_ElevatorMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    m_ElevatorLowerLimit.enableLimitSwitch(false);
    m_ElevatorUpperLimit.enableLimitSwitch(false);

    m_ElevatorPIDController.setP(Elevator.kP);
    m_ElevatorPIDController.setI(Elevator.kI);
    m_ElevatorPIDController.setD(Elevator.kD);
    m_ElevatorPIDController.setIZone(Elevator.kIz);
    m_ElevatorPIDController.setFF(Elevator.kFF);
    m_ElevatorPIDController.setOutputRange(Elevator.kMinOutput, Elevator.kMaxOutput);

    // Display PID cofficients for Elevator on Smartboard
    SmartDashboard.putNumber("P GainT", Elevator.kP);
    SmartDashboard.putNumber("I Gain", Elevator.kI);
    SmartDashboard.putNumber("D Gain", Elevator.kD);
    SmartDashboard.putNumber("I Zone", Elevator.kIz);
    SmartDashboard.putNumber("Feed Forward", Elevator.kFF);
    SmartDashboard.putNumber("Max Output", Elevator.kMaxOutput);
    SmartDashboard.putNumber("Min Output", Elevator.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Elevator Position", m_ElevatorEncoder.getPosition());
    SmartDashboard.putNumber("countsPerRev", m_ElevatorEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("RotsPerInch", Elevator.rotationsPerInch);
    SmartDashboard.putBoolean("Upper limit elevator", m_ElevatorUpperLimit.isPressed());
    SmartDashboard.putBoolean("lower limit elevator", m_ElevatorLowerLimit.isPressed());

    // SendableRegistry.setName(m_ElevatorPIDController,"elevatorSubsystem","Elevator");

    // NetworkTable table = NetworkTable.getTable
  }

  public void MoveElevator(double speed) { // moves elevator with a given speed
    m_ElevatorMotor.set(speed);
  }

  // function to move elevator to specific height
  public void MoveElevatorTo(double heightInches) {
    m_ElevatorPIDController.setReference(
        (heightInches * Elevator.rotationsPerInch),
        ControlType
            .kPosition); // TODO have to check if works, and conversion of units may need to happen
  }

  public double elevatorPos() {
    return m_ElevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double elavatorP = SmartDashboard.getNumber("elavator P Gain", Elevator.kP);
    double elavatorI = SmartDashboard.getNumber("elavator I Gain", Elevator.kI);
    double elavatorD = SmartDashboard.getNumber("elavator D Gain", Elevator.kD);
    double elavatorIzone = SmartDashboard.getNumber("elavator I Zone", Elevator.kIz);
    double elavatorff = SmartDashboard.getNumber("elavator Feed Forward", Elevator.kFF);
    double elavatorMax = SmartDashboard.getNumber("shooter pivot Max Output", Elevator.kMaxOutput);
    double elavatorMin = SmartDashboard.getNumber("shooter pivot Min Output", Elevator.kMinOutput);
    double elavatorRotations = SmartDashboard.getNumber("shooter pivot Set Rotations", 0);
    SmartDashboard.putNumber("Elevator Position", m_ElevatorEncoder.getPosition());
    SmartDashboard.putNumber("elevatorp out", elavatorP);
    SmartDashboard.putNumber("P GainT", elavatorP);
    SmartDashboard.putBoolean("Upper limit elevator", m_ElevatorUpperLimit.isPressed());
    SmartDashboard.putBoolean("lower limit elevator", m_ElevatorLowerLimit.isPressed());
    if ((elavatorP != Elevator.kP)) {
      m_ElevatorPIDController.setP(elavatorP);
      elavatorP = Elevator.kP;
    } // making the shooterpivot PID to match the PIDs on the Dashboard
    if ((elavatorI != Elevator.kI)) {
      m_ElevatorPIDController.setI(elavatorI);
      elavatorI = Elevator.kI;
    }
    if ((elavatorD != Elevator.kD)) {
      m_ElevatorPIDController.setD(elavatorD);
      elavatorD = Elevator.kD;
    }
    if ((elavatorIzone != Elevator.kIz)) {
      m_ElevatorPIDController.setIZone(elavatorIzone);
      elavatorIzone = Elevator.kIz;
    }
    if ((elavatorff != Elevator.kFF)) {
      m_ElevatorPIDController.setFF(elavatorff);
      elavatorff = Elevator.kFF;
    }
    if ((elavatorMax != Elevator.kMaxOutput) || (elavatorMin != Elevator.kMinOutput)) {
      // m_PivotPIDController.setOutputRange(elavatorMin, elavatorMax);

      elavatorMin = ShooterConstants.kMinOutput;
    }
  }
}
