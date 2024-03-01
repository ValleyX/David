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

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class RevCanIDs {

    // TODO have to get CanIDs
    public static final int kCAN_Roborio = 0;

    public static final int kCAN_TurnLeftFront = 1;
    public static final int kCAN_TurnLeftBack = 2;
    public static final int kCAN_TurnRightFront = 3;
    public static final int kCAN_TurnRightBack = 4;

    public static final int kCAN_DriveLeftFront = 5;
    public static final int kCAN_DriveLeftBack = 6;
    public static final int kCAN_DriveRightFront = 7;
    public static final int kCAN_DriveRightBack = 8;

    public static final int kCAN_PowerHub = 9;

    public static final int kCAN_ShooterMotor = 10;
    public static final int kCAN_ElevatorMotor = 11;

    public static final int kCAN_ClimberMotorLeft = 12;
    public static final int kCAN_ClimberMotorRight = 13;
    // public static final int kCAN_ClimberPivotSensor = 14;
    // public static final int kCAN_ClimberAngle = 22;

    public static final int kCAN_PivotMotor = 15;
    public static final int kCAN_FloorIntakeMotor = 16;
    public static final int kCAN_ShooterIntakeMotor = 17;
  }

  public static final class CTRECanIDs {

    public static final int kCAN_EncoderLeftFront = 1;
    public static final int kCAN_EncoderLeftBack = 2;
    public static final int kCAN_EncoderRightFront = 3;
    public static final int kCAN_EncoderRightBack = 4;

    public static final int kCAN_PigeonIMU = 5;

    // public static final int kCAN_PivotCANCoder = 23;

  }

  public static final class DigitalInputIDs {

    public static final int kDIG_ShootBeamBreak = 8;
    public static final int kDIG_FloorBeamBreak = 9;
    /*public static final int kDIG_ElevatorUpLimitSwitch = 2;
    public static final int kDIG_ElevatorDownLimitSwitch = 3;
    public static final int kDIG_ClimberDownLimitSwitch = 5;
    public static final int kDIG_PivotDownLimitSwitch =
        6; // it will not properly intake before touched*/
  }

  public static final class PWMIDs {

    public static final int kPWM_TowerBlinkin = 8;
    public static final int kPWM_BaseBlinkin = 9;
    /*public static final int kDIG_ElevatorUpLimitSwitch = 2;
    public static final int kDIG_ElevatorDownLimitSwitch = 3;
    public static final int kDIG_ClimberDownLimitSwitch = 5;
    public static final int kDIG_PivotDownLimitSwitch =
        6; // it will not properly intake before touched*/
  }

  public static final class ShooterConstants {

    public static final double shooterSpeed = 0.5;

    // Shooter Intake
    public static final double OutTake = -0.2;
    public static final double inTake = 0.2;

    // Shooter PID
    public static final double kP = 0.1;
    public static final double kI = 0.0001;
    public static final double kD = 1;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    public static final double kMaxSpeedMetersPerSecond = 20;
    public static final double kMaxAngularSpeed =
        12 * Math.PI; // radians per second // how fast robot turns

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Angular offsets of the modules relative to the chassis in radians
    /*
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    */

    /**
     * to find the offset open the "REV Hardware Client" application and plug in computer to the
     * Power Distribution hub then look at the SparkMAX absolute encoder that is related to the turn
     * motor that you want to offset square the wheel using provided bracket if the bracket alligned
     * wheel is facing forward write the number of the absolute value here if the bracket allignedd
     * wheel is facing left, write the number of the absolute value but minus PI/2 off
     */
    public static final double kFrontLeftChassisAngularOffset = 1.612 - (Math.PI / 2); // 1.600

    public static final double kFrontRightChassisAngularOffset = 4.2685;
    public static final double kBackLeftChassisAngularOffset = 6.253139 - Math.PI; // 3.145
    public static final double kBackRightChassisAngularOffset = 5.922 + (Math.PI / 2);

    /*
        // Drivetrain Talon FX CAN IDs
        public static final int kFrontLeftDrivingCanId = 5;
        public static final int kRearLeftDrivingCanId = 6;
        public static final int kFrontRightDrivingCanId = 8;
        public static final int kRearRightDrivingCanId = 7;

        // Drivetrain SPARK MAX CAN IDs
        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId = 2;
        public static final int kFrontRightTurningCanId = 4;
        public static final int kRearRightTurningCanId = 3;
    */
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; // 3 inches
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kCountsPerRev = 2048;
    public static final double kSecIn100MS = 0.1;
    public static final double k100MSinSec = 100;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04; // TODO have to tune Driving and turning PID
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    // public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    // public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class Elevator { // The NEO motor type
    public static final double countsPerMotorREV =
        1; // 7168; // found via RevRobotics site docs.revrobotics.com/brushless/neo/vortex
    public static final double gearReduction = 5.48251; // mat calculation
    public static final double gearDiameterIn = 1.592;
    public static final double rotationsPerInch =
        (countsPerMotorREV * gearReduction) / (gearDiameterIn * Math.PI);

    // Elevator PID
    public static final double kP = 1; // TODO have to tune Elevator PID
    public static final double kI = 0; // .0001
    public static final double kD = 0; // 1
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -.3;
  }

  public static final class Pivot { // The NEO motor type
    public static final double countsPerMotorREV =
        1; // found via RevRobotics site docs.revrobotics.com/brushless/neo/vortex
    public static final double gearReduction = 74.4; // mat calculation
    public static final double gearDiameterIn = 1;
    public static final double RotationsPerDegree = // .2328;
        (gearReduction) / (360);

    // Pivit PID
    public static final double kP = 1; // .5 // TODO have to tune Pivit PID
    public static final double kI = 0; // .00025
    // public static final double kP = 1; // TODO have to tune Pivit PID
    // public static final double kI = 0.000;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.4;
    public static final double kMinOutput = -0.4;
  }

  public static final class Climber { // Neo motor type

    public static final double kUpSpeed = 0.3;
    public static final double kDownSpeed = -0.3;
    public static final double kUpAngle = 45;
    public static final double kDownAngle = 10;

    public static final double countsPerMotorREV =
        1; // found via RevRobotics site docs.revrobotics.com/brushless/neo/vortex
    public static final double gearReduction = 100; // mat calculation
    public static final double gearDiameterIn = 1.25;
    public static final double RotationsPerDegree = // .2328;
        (gearReduction) / (gearDiameterIn * Math.PI);

    // Climber Pivot PID
    public static final double kP = 0.1; // TODO have to tune Climber PID
    public static final double kI = 0.0001;
    public static final double kD = 1;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
  }

  public static final class IntakeConstants {
    public static final double kInSpeed = 1;
    public static final double kOutSpeed = -0.4;
    public static final double kStop = 0;
  }

  public static final class ColorConstants {
    // public static final Color kPurpleTarget = new Color(0.211, 0.332, 0.456);
    public static final double colorConfidenceTreshold = 0.94;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int KManipulatorControllerPort = 1;
  }

  public static final class LightConstants {
    public static final double kViolet = 0.91;
    public static final double kGold = 0.67;
    public static final double kStrobeRed = -0.11;
    public static final double kBreathBlue = -0.15;
    public static final double kStrobeGold = -0.07; // indicating cone
    public static final double kStrobeBlue = -0.09; // indicating cube
    public static final double kJosh = 0.93; // color white
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 7000; // 5676
  }

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
