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
import frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkFlex m_LeftClimber;
  private final CANSparkFlex m_RightClimber;

  // private final SparkAbsoluteEncoder m_ClimberAbsoluteEncoder;

  // private final CANCoderJNI m_climberAngle;

  private final RelativeEncoder m_ClimberLeftEncoder;
  private final RelativeEncoder m_ClimberRightEncoder;

  private SparkPIDController m_ClimberPIDControllerLeft;
  private SparkPIDController m_ClimberPIDControllerRight;
  // private SparkLimitSwitch climberDown;
  private SparkLimitSwitch climberDownLeft;
  private SparkLimitSwitch climberDownRight;

  /** Creates a new Climber. */
  public ClimberSubsystem() {

    m_LeftClimber =
        new CANSparkFlex(
            RevCanIDs.kCAN_ClimberMotorLeft, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_RightClimber =
        new CANSparkFlex(
            RevCanIDs.kCAN_ClimberMotorRight,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    /*m_LeftClimber.restoreFactoryDefaults(); //
    m_RightClimber.restoreFactoryDefaults();

    //m_RightClimber.follow(m_LeftClimber, true); // makes the right motor follow what ever
    // m_RightClimber.setInverted(true); // inverts second motor

    */ m_ClimberLeftEncoder = m_LeftClimber.getEncoder(); // TODO have to check later
    m_ClimberRightEncoder = m_RightClimber.getEncoder();
    /*m_ClimberPIDController =
        m_LeftClimber.getPIDController(); // setting up PID controller from the motor
    m_ClimberPIDController.setFeedbackDevice(
        m_ClimberEncoder); // setting the encoder to feed back into the motor to make the motor
    // beable to go to specfic cordinates

    // TODO Theses could be normally closed or open
    // these are to tell the motor if you have hit the max or min(boolean) and the motor cuts off if
    // so it doesn't break
    // climberDown = m_LeftClimber.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    climberDownLeft = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    climberDownLeft.enableLimitSwitch(false);

    m_LeftClimber.setIdleMode(IdleMode.kBrake);
    m_RightClimber.setIdleMode(IdleMode.kBrake);

    // make absolute encoder
    */
    // m_ClimberAbsoluteEncoder =
    // m_LeftClimber.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    // sets position of absolute encoder
    /*m_ClimberEncoder.setPosition(
        Climber.RotationsPerDegree * (((m_ClimberAbsoluteEncoder.getPosition()) * 360)));

    // seting PID coeficients for climb motor
    m_ClimberPIDController.setP(Climber.kP);
    m_ClimberPIDController.setI(Climber.kI);
    m_ClimberPIDController.setD(Climber.kD);
    m_ClimberPIDController.setIZone(Climber.kIz);
    m_ClimberPIDController.setFF(Climber.kFF);
    m_ClimberPIDController.setOutputRange(Climber.kMinOutput, Climber.kMaxOutput);

    // Display PID cofficients for shooter pivot motor on Smartboard
    SmartDashboard.putNumber("P Gain", Climber.kP);
    SmartDashboard.putNumber("I Gain", Climber.kI);
    SmartDashboard.putNumber("D Gain", Climber.kD);
    SmartDashboard.putNumber("I Zone", Climber.kIz);
    SmartDashboard.putNumber("Feed Forward", Climber.kFF);
    SmartDashboard.putNumber("Max Output", Climber.kMaxOutput);
    SmartDashboard.putNumber("Min Output", Climber.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("position", m_ClimberEncoder.getPosition());*/
    initUnfollow();
  }

  // moves climber to specific position
  public void moveClimberTo(double positionFromBottomDeg) {
    m_ClimberPIDControllerLeft.setReference(
        positionFromBottomDeg * Climber.RotationsPerDegree, ControlType.kPosition);

    m_ClimberPIDControllerRight.setReference(
        positionFromBottomDeg * Climber.RotationsPerDegree, ControlType.kPosition);
  }

  public void resetEncoders() {
    m_ClimberLeftEncoder.setPosition(0);
    m_ClimberRightEncoder.setPosition(0);
  }

  // returns position of climbers
  public double climberPosLeft() {
    return m_ClimberLeftEncoder.getPosition();
  }

  // returns position of climbers
  public double climberPosRight() {
    return m_ClimberRightEncoder.getPosition();
  }

  /** Extends the climber motors */
  public void Extend() {
    m_LeftClimber.set(Climber.kUpSpeed);
  }

  /** Retracts the climber motors */
  public void Retract() {
    m_LeftClimber.set(Climber.kDownSpeed);
  }

  public void moveClimbers(double speed) {
    if (Math.abs(speed) > 0.1) {
      m_LeftClimber.set(speed);
    } else {
      m_LeftClimber.set(0);
    }
  }

  public void moveBothClimbers(double speed) {
    if ((speed < -0.1) || ((speed > 0.1) && (m_ClimberLeftEncoder.getPosition() < 270))) {
      m_LeftClimber.set(speed);
    } else {
      m_LeftClimber.set(0);
    }

    if (IsDownLeft()) {
      m_ClimberLeftEncoder.setPosition(0);
    }

    if ((speed < -0.1) || ((speed > 0.1) && (m_ClimberRightEncoder.getPosition() < 270))) {
      m_RightClimber.set(speed);
    } else {
      m_RightClimber.set(0);
    }

    if (IsDownRight()) {
      m_ClimberRightEncoder.setPosition(0);
    }
  }

  public void GoToAngle(double degrees) {
    m_ClimberPIDControllerLeft.setReference(
        degrees,
        ControlType
            .kPosition); // TODO have to check if works, and conversion of units may need to happen
  }

  /**
   * gets the absolute encoder position from the pivot climb motor
   *
   * @return a double in motor in degrees
   */
  public double GetAngle() {
    return m_LeftClimber
        .getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle)
        .getPosition(); // TODO check if works what outputs
  }
  /**
   * checks if the climber limit switch has been pressed
   *
   * @returns boolean
   */
  public boolean IsDownLeft() { // checks limit switch to see if climber is down
    return climberDownLeft.isPressed();
    // return false;
  }

  public boolean IsDownRight() { // checks limit switch to see if climber is down
    return climberDownRight.isPressed();
    // return false;
  }
  // causes right motor to follow left motor
  public void initFollow() {

    m_LeftClimber.restoreFactoryDefaults(); //
    m_RightClimber.restoreFactoryDefaults();

    m_RightClimber.follow(m_LeftClimber, true); // makes the right motor follow what ever
    // m_RightClimber.setInverted(true); // inverts second motor

    // m_ClimberEncoder = m_LeftClimber.getEncoder(); // TODO have to check later
    m_ClimberPIDControllerLeft =
        m_LeftClimber.getPIDController(); // setting up PID controller from the motor
    m_ClimberPIDControllerLeft.setFeedbackDevice(
        m_ClimberLeftEncoder); // setting the encoder to feed back into the motor to make the motor
    // beable to go to specfic cordinates

    // TODO Theses could be normally closed or open
    // these are to tell the motor if you have hit the max or min(boolean) and the motor cuts off if
    // so it doesn't break
    // climberDown = m_LeftClimber.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    climberDownLeft = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    climberDownLeft.enableLimitSwitch(false);

    m_LeftClimber.setIdleMode(IdleMode.kBrake);
    m_RightClimber.setIdleMode(IdleMode.kBrake);

    // make absolute encoder
    // m_ClimberAbsoluteEncoder =
    //  m_LeftClimber.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    // sets position of absolute encoder
    // m_ClimberLeftEncoder.setPosition(
    //    Climber.RotationsPerDegree * (((m_ClimberAbsoluteEncoder.getPosition()) * 360)));

    // seting PID coeficients for climb motor
    m_ClimberPIDControllerLeft.setP(Climber.kP);
    m_ClimberPIDControllerLeft.setI(Climber.kI);
    m_ClimberPIDControllerLeft.setD(Climber.kD);
    m_ClimberPIDControllerLeft.setIZone(Climber.kIz);
    m_ClimberPIDControllerLeft.setFF(Climber.kFF);
    m_ClimberPIDControllerLeft.setOutputRange(Climber.kMinOutput, Climber.kMaxOutput);

    // Display PID cofficients for shooter pivot motor on Smartboard
    SmartDashboard.putNumber("P Gain", Climber.kP);
    SmartDashboard.putNumber("I Gain", Climber.kI);
    SmartDashboard.putNumber("D Gain", Climber.kD);
    SmartDashboard.putNumber("I Zone", Climber.kIz);
    SmartDashboard.putNumber("Feed Forward", Climber.kFF);
    SmartDashboard.putNumber("Max Output", Climber.kMaxOutput);
    SmartDashboard.putNumber("Min Output", Climber.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("position", m_ClimberLeftEncoder.getPosition());
  }

  public void initUnfollow() {
    m_LeftClimber.restoreFactoryDefaults();
    m_RightClimber.restoreFactoryDefaults();

    m_RightClimber.setInverted(true);

    m_ClimberPIDControllerLeft =
        m_LeftClimber.getPIDController(); // setting up PID controller from the motor
    m_ClimberPIDControllerLeft.setFeedbackDevice(m_ClimberLeftEncoder);
    m_ClimberLeftEncoder.setPosition(0);

    m_ClimberPIDControllerRight =
        m_RightClimber.getPIDController(); // setting up PID controller from the motor
    m_ClimberPIDControllerRight.setFeedbackDevice(m_ClimberRightEncoder);
    m_ClimberRightEncoder.setPosition(0);

    climberDownLeft = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    climberDownLeft.enableLimitSwitch(true);

    climberDownRight = m_RightClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    climberDownRight.enableLimitSwitch(true);

    m_LeftClimber.setIdleMode(IdleMode.kBrake);
    m_RightClimber.setIdleMode(IdleMode.kBrake);

    m_ClimberPIDControllerLeft.setP(Climber.kP);
    m_ClimberPIDControllerLeft.setI(Climber.kI);
    m_ClimberPIDControllerLeft.setD(Climber.kD);
    m_ClimberPIDControllerLeft.setIZone(Climber.kIz);
    m_ClimberPIDControllerLeft.setFF(Climber.kFF);
    m_ClimberPIDControllerLeft.setOutputRange(Climber.kMinOutput, Climber.kMaxOutput);

    m_ClimberPIDControllerRight.setP(Climber.kP);
    m_ClimberPIDControllerRight.setI(Climber.kI);
    m_ClimberPIDControllerRight.setD(Climber.kD);
    m_ClimberPIDControllerRight.setIZone(Climber.kIz);
    m_ClimberPIDControllerRight.setFF(Climber.kFF);
    m_ClimberPIDControllerRight.setOutputRange(Climber.kMinOutput, Climber.kMaxOutput);
  }

  @Override
  public void periodic() {
    // TODO turn climbers off when they reach their given encoder points

    // getting PID values from smartDashBoard
    double climberPivotP = SmartDashboard.getNumber("climberPivotP Gain", Climber.kP);
    double climberPivotI = SmartDashboard.getNumber("climberPivotI Gain", Climber.kI);
    double climberPivotD = SmartDashboard.getNumber("climberPivotD Gain", Climber.kD);
    double climberPivotIzone = SmartDashboard.getNumber("climberPivot I Zone", Climber.kIz);
    double climberPivotff = SmartDashboard.getNumber("climberPivot Feed Forward", Climber.kFF);
    double climberPivotMax =
        SmartDashboard.getNumber("climberPivot Max Output", Climber.kMaxOutput);
    double climberPivotMin =
        SmartDashboard.getNumber("climberPivot Min Output", Climber.kMinOutput);
    double climberPivotRotations = SmartDashboard.getNumber("climberPivot Set Rotations", 0);
    // SmartDashboard.putBoolean("switch Down settting", climberDown.isPressed());
    SmartDashboard.putBoolean("switch Up settting", climberDownLeft.isPressed());
    SmartDashboard.putNumber("Climber Left encoder", m_ClimberLeftEncoder.getPosition());
    SmartDashboard.putNumber("Climber Right encoder", m_ClimberRightEncoder.getPosition());
    // making sure PID loop values match the smartboard values
    if ((climberPivotP != Climber.kP)) {
      m_ClimberPIDControllerLeft.setP(climberPivotP);
      climberPivotP = Climber.kP;
    }
    if ((climberPivotI != Climber.kI)) {
      m_ClimberPIDControllerLeft.setI(climberPivotI);
      climberPivotI = Climber.kI;
    }
    if ((climberPivotD != Climber.kD)) {
      m_ClimberPIDControllerLeft.setD(climberPivotD);
      climberPivotD = Climber.kD;
    }
    if ((climberPivotIzone != Climber.kIz)) {
      m_ClimberPIDControllerLeft.setIZone(climberPivotIzone);
      climberPivotIzone = Climber.kIz;
    }
    if ((climberPivotff != Climber.kFF)) {
      m_ClimberPIDControllerLeft.setFF(climberPivotff);
      climberPivotff = Climber.kFF;
    }
    if ((climberPivotMax != Climber.kMaxOutput) || (climberPivotMin != Climber.kMinOutput)) {
      m_ClimberPIDControllerLeft.setOutputRange(climberPivotMin, climberPivotMax);
      climberPivotMax = Climber.kMaxOutput;
      climberPivotMin = Climber.kMinOutput;
    }

    // m_ClimberPIDController.setReference(
    //    climberPivotRotations, ControlType.kPosition); // This makes the PID loop go
  }
}
