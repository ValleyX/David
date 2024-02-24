// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pivot;
import frc.robot.Constants.RevCanIDs;
import java.util.function.BooleanSupplier;

public class PivotSubsystem extends SubsystemBase {
  private final CANSparkFlex m_ShooterPivotMotor; // Used to aim shooter
  private final SparkAbsoluteEncoder m_PivotAbsoluteEncoder;
  private final RelativeEncoder m_PivotEncoder;
  private BooleanSupplier test;

  private SparkPIDController m_PivotPIDController;
  private SparkLimitSwitch m_PivotIntakeLimitSwitch;
  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    m_ShooterPivotMotor =
        new CANSparkFlex(
            RevCanIDs.kCAN_PivotMotor,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless); // have PID loop

    m_ShooterPivotMotor.restoreFactoryDefaults();
    m_ShooterPivotMotor.setInverted(false);
    // m_ShooterPivotMotor.setIdleMode(IdleMode.kBrake); // setting motor so it make it breaks in
    // place
    // m_ShooterPivotMotor.getAbsoluteEncoder()

    // m_PivotEncoder =
    // m_ShooterPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_PivotEncoder = m_ShooterPivotMotor.getEncoder();
    m_PivotPIDController =
        m_ShooterPivotMotor.getPIDController(); // setting up PID controller from the motor
    m_PivotPIDController.setFeedbackDevice(
        m_PivotEncoder); // setting the encoder to feed back into the motor to make the moto

    m_PivotIntakeLimitSwitch =
        m_ShooterPivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // m_PivotEncoder.setInverted(true);
    // m_PivotEncoder.setZeroOffset((1-0.920) + (60/360));
    m_PivotAbsoluteEncoder =
        m_ShooterPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    // m_PivotEncoder.setZeroOffset(0.827);

    m_PivotEncoder.setPosition(
        Pivot.RotationsPerDegree * (180 - (m_PivotAbsoluteEncoder.getPosition() * 360)));
    // seting PID coeficients for pivot motor

    m_PivotPIDController.setP(Pivot.kP);
    m_PivotPIDController.setI(Pivot.kI);
    m_PivotPIDController.setD(Pivot.kD);
    m_PivotPIDController.setIZone(Pivot.kIz);
    m_PivotPIDController.setFF(Pivot.kFF);
    m_PivotPIDController.setOutputRange(Pivot.kMinOutput, Pivot.kMaxOutput);

    // Display PID cofficients for shooter pivot motor on Smartboard
    SmartDashboard.putNumber("pivot P Gain", Pivot.kP);
    SmartDashboard.putNumber("pivot I Gain", Pivot.kI);
    SmartDashboard.putNumber("pivot D Gain", Pivot.kD);
    SmartDashboard.putNumber("pivot I Zone", Pivot.kIz);
    SmartDashboard.putNumber("pivot Feed Forward", Pivot.kFF);
    SmartDashboard.putNumber("pivot Max Output", Pivot.kMaxOutput);
    SmartDashboard.putNumber("pivot Min Output", Pivot.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Rotations of Pivot: ", m_PivotEncoder.getPosition());
    SmartDashboard.putNumber(
        "Degrees of Pivot: ", m_PivotEncoder.getPosition() / Pivot.RotationsPerDegree);
    SmartDashboard.putNumber("Absolute Encoder Pivot", m_PivotAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber(
        "Absolute Encoder Pivot Degrees", m_PivotAbsoluteEncoder.getPosition() * 360);
    // SmartDashboard.putData(m_PivotPIDController);
    // addChild("Test", m_PivotEncoder);
    test = () -> m_PivotIntakeLimitSwitch.isLimitSwitchEnabled();
  }

  public void MovePivotTo(double positionDeg) { // Move Pivot motor to specific location by PID loop
    // m_PivotPIDControllerm_PivotAbsoluteEncoder.getPosition() * 360)) / Pivot.RotationsPerDegree
    // TODO take degrees to rotations

    m_PivotPIDController.setReference(
        (positionDeg * Pivot.RotationsPerDegree),
        ControlType.kPosition); // TODO check if works and put rotations as input
  }

  public double pivotPos() {
    return m_PivotEncoder.getPosition();
  }

  public void MovePivot(double speed) { // moves pivot with a given speed
    if (Math.abs(speed) > 0.1) {
      m_ShooterPivotMotor.set(speed);
    } else {
      m_ShooterPivotMotor.set(0);
    }
  }

  public boolean IsShooterAligned() { // checks if pivot limit switch is active from the motors
    return (m_PivotAbsoluteEncoder.getPosition() * 360) < 162;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Display PID cofficients for shooter pivot motor on Smartboard
    double shooterPivotP = SmartDashboard.getNumber("shooterPivotP Gain", Pivot.kP);
    double shooterPivotI = SmartDashboard.getNumber("shooterPivotI Gain", Pivot.kI);
    double shooterPivotD = SmartDashboard.getNumber("shooterPivotD Gain", Pivot.kD);
    double shooterPivotIzone = SmartDashboard.getNumber("shooterPivot I Zone", Pivot.kIz);
    double shooterPivotff = SmartDashboard.getNumber("shooter pivot Feed Forward", Pivot.kFF);
    double shooterPivotMax = SmartDashboard.getNumber("shooter pivot Max Output", Pivot.kMaxOutput);
    double shooterPivotMin = SmartDashboard.getNumber("shooter pivot Min Output", Pivot.kMinOutput);
    double shooterPivotRotations = SmartDashboard.getNumber("shooter pivot Set Rotations", 0);
    SmartDashboard.putNumber("Rotations of Pivot: ", m_PivotEncoder.getPosition());
    SmartDashboard.putNumber("Absolute Encoder Pivot", m_PivotAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber(
        "Degrees of Pivot: ", m_PivotEncoder.getPosition() / Pivot.RotationsPerDegree);
    SmartDashboard.putNumber(
        "Absolute Encoder Pivot Degrees", m_PivotAbsoluteEncoder.getPosition() * 360);
    SmartDashboard.putNumber(
        "Adjusted Absolute Encoder Pivot Degrees",
        (m_PivotAbsoluteEncoder.getPosition() - 0.25) * 360);

    if ((shooterPivotP != Pivot.kP)) {
      m_PivotPIDController.setP(shooterPivotP);
      shooterPivotP = Pivot.kP;
    } // making the shooterpivot PID to match the PIDs on the Dashboard
    if ((shooterPivotI != Pivot.kI)) {
      m_PivotPIDController.setI(shooterPivotI);
      shooterPivotI = Pivot.kI;
    }
    if ((shooterPivotD != Pivot.kD)) {
      m_PivotPIDController.setD(shooterPivotD);
      shooterPivotD = Pivot.kD;
    }
    if ((shooterPivotIzone != Pivot.kIz)) {
      m_PivotPIDController.setIZone(shooterPivotIzone);
      shooterPivotIzone = Pivot.kIz;
    }
    if ((shooterPivotff != Pivot.kFF)) {
      m_PivotPIDController.setFF(shooterPivotff);
      shooterPivotff = Pivot.kFF;
    }
    if ((shooterPivotMax != Pivot.kMaxOutput) || (shooterPivotMin != Pivot.kMinOutput)) {
      m_PivotPIDController.setOutputRange(shooterPivotMin, shooterPivotMax);
      shooterPivotMax = Pivot.kMaxOutput;
      shooterPivotMin = Pivot.kMinOutput;
    }
    // m_PivotPIDController.setReference(
    //   shooterPivotRotations, ControlType.kPosition); // This makes the PID loop go
  }
}
