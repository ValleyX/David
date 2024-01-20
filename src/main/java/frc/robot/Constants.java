// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CanIDs {

    public static final int kCAN_Roborio = 0;

    public static final int kCAN_TurnLeftFront = 1;
    public static final int kCAN_TurnLeftBack = 2;
    public static final int kCAN_TurnRightFront = 3;
    public static final int kCAN_TurnRightBack = 4;

    public static final int kCAN_DriveLeftFront = 5;
    public static final int kCAN_DriveLeftBack = 6;
    public static final int kCAN_DriveRightFront = 7;
    public static final int kCAN_DriveRightBack = 8;

    public static final int kCAN_EncoderLeftFront = 9;
    public static final int kCAN_EncoderLeftBack = 10;
    public static final int kCAN_EncoderRightFront = 11;
    public static final int kCAN_EncoderRightBack = 12;

    public static final int kCAN_PigeonIMU = 13;

    public static final int kCAN_PowerHub = 14;

    public static final int kCAN_ShooterMotor = 15;
    public static final int kCAN_ShooterIntakeMotor = 16;
    public static final int kCAN_ElevatorMotor = 17;
    public static final int kCAN_FloorIntakeMotor = 18;
    public static final int kCAN_ClimberMotorLeft = 19;
    public static final int kCAN_ClimberMotorRight = 20;
    public static final int kCAN_PivotCANCoder = 21;
    public static final int kCAN_PivotMotor = 22;


  } 

  public static final class DigitalInputIDs {

    public static final int kDIG_ShootBeanBreak = 0;

    public static final int kDIG_FloorBeanBreak = 1;

    public static final int kDIG_ElevatorUpLimitSwitch = 2;
    public static final int kDIG_ElevatorDownLimitSwitch = 3;

    public static final int kDIG_ClimberUpLimitSwitch = 4;
    public static final int kDIG_ClimberDownLimitSwitch = 5;

    public static final int kDIG_PivotDownLimitSwitch = 6; //it will not properly intake before touched
    
  
  }

  public static final class ShooterPivotPID{
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
    public static final double kMaxAngularSpeed = 12 * Math.PI; // radians per second // how fast robot turns

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(31);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    /* 
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    */

    /**
     * to find the offset open the "REV Hardware Client" application and plug in computer to the Power Distribution hub
     * then look at the SparkMAX absolute encoder that is related to the turn motor that you want to offset
     * square the wheel using provided bracket 
     * if the bracket alligned wheel is facing forward write the number of the absolute value here
     * if the bracket allignedd wheel is facing left, write the number of the absolute value but minus PI/2 off
     */
    public static final double kFrontLeftChassisAngularOffset = 1.612 - (Math.PI/2); //1.600
    public static final double kFrontRightChassisAngularOffset = 4.2685; 
    public static final double kBackLeftChassisAngularOffset = 6.253139 - Math.PI; //3.145
    public static final double kBackRightChassisAngularOffset = 5.922 + (Math.PI/2);

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
    public static final double kWheelDiameterMeters = 0.0762; //3 inches
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kCountsPerRev = 2048;
    public static final double kSecIn100MS = 0.1;
    public static final double k100MSinSec = 100;
    
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
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

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class ElevatorConstants{ //The NEO motor type
    public final static double countsPerMotorREV = 42; //found via RevRobotics site revrobotics.com/rev-21-1650/
    public final static double gearReduction = 5.48251; //mat calculation 
    public final static double gearDiameter = 1.592; //inches 
    public final static double countsPerInch = (countsPerMotorREV*gearReduction)/(gearDiameter*Math.PI);

    public static final double kP = 0.1;
    public static final double kI = 0.0001;
    public static final double kD = 1;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;


  }

  public static final class ClimberConstants { //Neo motor type

    public final static double kClimberUpSpeed = 0.3;
    public final static double kClimberDownSpeed = -0.3;


  }

  public static final class IntakeConstants {

    public final static double kIntakeInSpeed = 0.4;
    public final static double kExtakeOutSpeed = -0.4;
    public final static double kIntakeStop = 0;
  }

  public static final class ColorConstants {

    public final static Color kPurpleTarget = new Color(0.211, 0.332, 0.456);
    public final static double colorConfidenceTreshold = 0.94;

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
    public static final double kMaxSpeedMetersPerSecond = 3 ;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3  ;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =  Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 7000; //5676
  }
}
