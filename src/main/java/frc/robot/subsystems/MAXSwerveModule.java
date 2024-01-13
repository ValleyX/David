// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.CANSparkFlex;



public class MAXSwerveModule {

  private final CANSparkFlex m_drivingSparkFlex;
  private final CANSparkFlex m_turningSparkFlex;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {

    m_drivingSparkFlex = new CANSparkFlex(drivingCANId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_turningSparkFlex = new CANSparkFlex(turningCANId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    m_turningSparkFlex.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkFlex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_turningPIDController = m_turningSparkFlex.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    //m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    //m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
/* 
    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);
*/
    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

        
   // m_drivingTalonFX.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkFlex.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    //m_drivingTalonFX.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkFlex.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    //m_drivingTalonFX.burnFlash();
    m_turningSparkFlex.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    //m_drivingEncoder.setPosition(0);
    
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

     
    
    return new SwerveModuleState(m_drivingSparkFlex.getEncoder().getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));

       // return new SwerveModuleState(m_drivingSparkFlex.get(),
       // new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));


       // return new SwerveModuleState(m_drivingEncoder.getVelocity(),
       // new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(

    m_drivingSparkFlex.getEncoder().getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
/* 
      m_drivingTalonFX.getSelectedSensorPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));

      */

     //   return new SwerveModulePosition(
     //     m_drivingEncoder.getPosition(),
     //     new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public double toCountsPerMS(double MetersPerSec) {
    return (MetersPerSec * Constants.ModuleConstants.kCountsPerRev / Constants.ModuleConstants.kSecIn100MS)
    /(Constants.ModuleConstants.kWheelCircumferenceMeters * Constants.ModuleConstants.k100MSinSec);

  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
/* 
    if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 0.001){
        stop();
        return;
    }
*/
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    //m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
   // m_drivingTalonFX.set(ControlMode.Velocity, toCountsPerMS(optimizedDesiredState.speedMetersPerSecond));
    m_drivingSparkFlex.setControlFramePeriodMs((int)toCountsPerMS(optimizedDesiredState.speedMetersPerSecond)); //JM not sure about this conversion
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("optimizedDesiredState.speedMetersPerSecond", optimizedDesiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Speed countsPerMS", toCountsPerMS(optimizedDesiredState.speedMetersPerSecond));

    SmartDashboard.putNumber("optimizedDesiredState.angle.getRadians()", optimizedDesiredState.angle.getRadians());
    SmartDashboard.putNumber("optimizedDesiredState.angle.getDegrees", optimizedDesiredState.angle.getDegrees());

   // SmartDashboard.putNumber("GetPosition " + m_turningSparkMax.getDeviceId(), getPosition().angle.getDegrees());
    SmartDashboard.putNumber("GetPosition " + m_turningSparkFlex.getDeviceId(), getPosition().angle.getDegrees());

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    //m_drivingEncoder.setPosition(0);
    //m_drivingTalonFX.setSelectedSensorPosition(0);
    m_drivingSparkFlex.getEncoder().setPosition(0);
  }

/* 
  public WPI_TalonFX getDriveMotor(){
    return m_drivingTalonFX;
  }
*/
  public CANSparkFlex getDriveMotor(){
    return m_drivingSparkFlex;
  }

/* 
  public CANSparkMax getTurnMotor(){
    return m_turningSparkMax;
  }
  */

   public CANSparkFlex getTurnMotor(){
    return m_turningSparkFlex;
  }
 

  public void stop(){
    //m_drivingTalonFX.set(0);
    //m_turningSparkMax.set(0);

    m_drivingSparkFlex.set(0);
    m_turningSparkFlex.set(0);
  }
  
}
