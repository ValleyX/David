// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlexFx implements ModuleIO {

  private final TalonFX driveTalonFX;
  private final CANSparkFlex turnSparkMax;

  // private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  // private final AnalogInput turnAbsoluteEncoder;
  public final CANcoder turnAbsoluteEncoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private boolean isTurnMotorInverted = true;
  // Gear ratios for SDS MK4i L2, adjust as necessary
  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module#:~:text=The%20steering%20gear%20ratio%20of,standard%20full%20weight%20competition%20robots.
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;
  private boolean isDriverMotorInverted = false;
  // private final Rotation2d absoluteEncoderOffset;
  private final double absoff;
  // private final StatusSignal<Double> turnAbsolutePosition;

  public ModuleIOSparkFlexFx(int index) {
    switch (index) {
      case 0: // LF
        // driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        // turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        driveTalonFX = new TalonFX(Constants.RevCanIDs.kCAN_DriveLeftFront);
        // driveSparkMax.setInverted(false);
        turnSparkMax =
            new CANSparkFlex(Constants.RevCanIDs.kCAN_TurnLeftFront, MotorType.kBrushless);
        //  turnSparkMax.setInverted(true);
        // isTurnMotorInverted = false;
        isDriverMotorInverted = true;

        turnAbsoluteEncoder = new CANcoder(Constants.CTRECanIDs.kCAN_EncoderLeftFront, "CTRCAN");

        // turnAbsoluteEncoder = new AnalogInput(0);
        //     absoluteEncoderOffset = new Rotation2d(-0.210693 * 2 * Math.PI); // MUST BE
        // CALIBRATED
        // absoff = -0.3271;
        break;
      case 1: // RF
        driveTalonFX = new TalonFX(Constants.RevCanIDs.kCAN_DriveRightFront);
        // driveSparkMax.setInverted(true);
        turnSparkMax =
            new CANSparkFlex(Constants.RevCanIDs.kCAN_TurnRightFront, MotorType.kBrushless);
        isTurnMotorInverted = false;
        // turnSparkMax.setInverted(true);
        turnAbsoluteEncoder = new CANcoder(Constants.CTRECanIDs.kCAN_EncoderRightFront, "CTRCAN");
        // absoluteEncoderOffset = new Rotation2d(0.2015 * 2 * Math.PI); // MUST BE CALIBRATED
        // absoff = 0.2563;

        // absoluteEncoderOffset = new Rotation2d(index)
        break;
      case 2: // LB
        driveTalonFX = new TalonFX(Constants.RevCanIDs.kCAN_DriveLeftBack);
        turnSparkMax =
            new CANSparkFlex(Constants.RevCanIDs.kCAN_TurnLeftBack, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(Constants.CTRECanIDs.kCAN_EncoderLeftBack, "CTRCAN");
        // absoluteEncoderOffset = new Rotation2d(-0.289307 * 2 * Math.PI); // MUST BE CALIBRATED
        // absoff = -0.388184;
        break;
      case 3: // RB
        driveTalonFX = new TalonFX(Constants.RevCanIDs.kCAN_DriveRightBack);
        turnSparkMax =
            new CANSparkFlex(Constants.RevCanIDs.kCAN_TurnRightBack, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(Constants.CTRECanIDs.kCAN_EncoderRightBack, "CTRCAN");
        // absoluteEncoderOffset = new Rotation2d(0.213623 * 2 * Math.PI); // MUST BE CALIBRATED
        // absoff = 0.201416;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    absoff = 0.0;
    // driveTalonFX.restoreFactoryDefaults();
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalonFX.getConfigurator().apply(driveConfig);

    setDriveBrakeMode(true);

    turnSparkMax.restoreFactoryDefaults();

    drivePosition = driveTalonFX.getPosition();
    driveVelocity = driveTalonFX.getVelocity();
    driveAppliedVolts = driveTalonFX.getMotorVoltage();
    driveCurrent = driveTalonFX.getStatorCurrent();

    // driveTalonFX.setCANTimeout(250); // 250
    turnSparkMax.setCANTimeout(250);

    // driveEncoder = driveTalonFX.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    // turnAbsolutePosition = turnAbsoluteEncoder.getAbsolutePosition();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveTalonFX.setInverted(isDriverMotorInverted);
    // driveTalonFX.setSmartCurrentLimit(35); // 40
    turnSparkMax.setSmartCurrentLimit(30);
    // driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    // driveEncoder.setPosition(0.0);
    // driveEncoder.setMeasurementPeriod(10);
    // driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    // driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    // driveTalonFX.burnFlash();
    turnSparkMax.burnFlash();
    setDriveBrakeMode(true);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);
    driveTalonFX.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs, int index) {
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
    /*
        SmartDashboard.putNumber("driveAppliedVolts " + index, inputs.driveAppliedVolts);
        SmartDashboard.putNumber("getAppliedOutput " + index, driveTalonFX.getAppliedOutput());
        SmartDashboard.putNumber("getBusVoltage " + index, driveTalonFX.getBusVoltage());

        SmartDashboard.putNumber("driveTemp" + index, driveTalonFX.getMotorTemperature());
    */
    /*
    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * (Math.PI / 180))
            .minus(absoluteEncoderOffset);
            */
    // inputs.turnAbsolutePosition = absoluteEncoderOffset;
    /*
        if (index == 1) // front right
        {
          inputs.turnAbsolutePosition =
              new Rotation2d(
                  (-turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() - absoff)
                      * 2.0
                      * Math.PI);
        } else {
    */
    inputs.turnAbsolutePosition =
        new Rotation2d(
            (turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() - absoff)
                * 2.0
                * Math.PI);
    //    }

    // SmartDashboard.putNumber(
    //    "hard ab with off " + index, inputs.turnAbsolutePosition.getRotations());
    // SmartDashboard.putNumber("off " + index, absoluteEncoderOffset.getRotations());
    // SmartDashboard.putNumber(
    //    "offrot " + index, turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() - absoff);

    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalonFX.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {

    var config = new MotorOutputConfigs();
    config.Inverted =
        isDriverMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalonFX.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
