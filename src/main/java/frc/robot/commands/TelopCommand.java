// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import com.fasterxml.jackson.databind.Module.SetupContext;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoyReadWrite;
import frc.robot.JoyStorage;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class TelopCommand extends Command {

    //private Joystick m_joystickDriver;
    public enum DriveType
    {
        Telop,
        Record,
        Playback
    }
 
    private int executeCount;
    private int m_seconds;
    private DriveType m_driveType;

    //calls in all of the subsystems
    private final DriveSubsystem m_robotDrive;
    private final XboxController m_driverController;
    private final XboxController m_manipulatorController;
    private double m_speedDivisor;
    private double m_speedDivisorRotation;

    private final Timer m_fieldTimer;

    //storage for input values, used for playback for recording autonomous
    private JoyStorage m_joy[];
    private final int m_joyCountmax = 750;
    String m_filename;
    private SlewRateLimiter m_SlewX;
    private SlewRateLimiter m_SlewY;

//this is the construcor for the teleop command class

    //int seconds and filename only used in record and playback scenarios
    /*
    driveType = determines whether the program is recording or playing back a recording
    seconds = how long the playback system records / plays
    filename = name of the playback file
    subsystem = drive subsystem (swerve)
    liftsub = subsystem to run the lift
    clawsub = subsytem to run the claw
    */
    public TelopCommand(DriveType drivetype, int seconds, String filename, DriveSubsystem subsystem) {

    //creates manipulator and driver controlers
     m_driverController = new XboxController(OIConstants.kDriverControllerPort);
     m_manipulatorController = new XboxController(OIConstants.KManipulatorControllerPort);

     m_fieldTimer = new Timer();

     m_SlewX = new SlewRateLimiter(2);
     m_SlewY = new SlewRateLimiter(2);

      
     //maps the subsystems to class variables
        m_robotDrive = subsystem;
        m_seconds = seconds;
        m_driveType = drivetype;
        m_filename = filename;
        addRequirements(m_robotDrive);
        
        //pushes duration and whether the system is doing playback or recording to the smart dashboard
        SmartDashboard.putNumber("seconds: ", seconds);
        SmartDashboard.putBoolean("is Playback", m_driveType == DriveType.Playback);

        executeCount = 0;

        m_speedDivisor = 1;
        m_speedDivisorRotation = 1;
    }

    // Called when the command is initially scheduled.
    //initializes all the stuff, only runs one time
    @Override
    public void initialize() {
      
        m_fieldTimer.reset();
        SmartDashboard.putString("DriveCommand init", "In init tryplayback");

    //determines if the program is reading or writing to joyStorage (in playback or record mode)
      if (m_driveType == DriveType.Playback)
      {
          SmartDashboard.putString("DriveCommand", "In playback");
          //load data from filename
          m_joy = new JoyStorage[m_joyCountmax];
          m_joy = JoyReadWrite.readObject(m_filename);
          SmartDashboard.putString("in playback", m_filename);
         // m_joy[0].leftdriveYstick = .2;
         
      }
      else
      {
          m_joy = new JoyStorage[m_joyCountmax];
      }

      
    
    }



    // Called every time the scheduler runs while the command is scheduled.

    //runs every 20 milliseconds to detect inputs and detectors on the robot (limit switches and breaks as well as manipulator and driver inputs)
    @Override
    public void execute() {

        double driveLeftYstick;
        double driveLeftXstick;
        double driveRightXstick;

        double manipulatorLeftYstick;
    

        //playback mode (reads from joyStorage)
        if ((executeCount >= 750) && (m_driveType != DriveType.Telop))
        {
            return;
        }

        //normal TeleOp (if not in playback mode, go to normal TeleOp controls) All buttons and driver inputs
        if (m_driveType != DriveType.Playback) {

            driveLeftYstick = m_driverController.getLeftY();
            driveLeftXstick = m_driverController.getLeftX();
            driveRightXstick = m_driverController.getRightX();

            manipulatorLeftYstick = m_manipulatorController.getLeftY();
          
            //if the drive type is in record, it records input values and writes them to joyStorage
            if (m_driveType == DriveType.Record){
               
                driveLeftYstick /= 2.5;
                driveLeftXstick /= 2.5;
                driveRightXstick /= 2.5;

                double yaw = m_robotDrive.m_gyro.getYaw();
                
                //keep bot straight during record
                driveRightXstick = yaw/20;
                
                m_joy[executeCount] =  new JoyStorage(driveLeftYstick, driveLeftXstick, driveRightXstick, manipulatorLeftYstick);

            }

        } 

        //this is where it plays back from joyStorage
        else 
        {
            driveLeftYstick = m_joy[executeCount].leftdriveYstick;
            driveLeftXstick = m_joy[executeCount].leftdriveXStick;
            driveRightXstick = m_joy[executeCount].rightdriveXstick;
            manipulatorLeftYstick = m_joy[executeCount].manipulatorLeftYstick;

            double yaw = m_robotDrive.m_gyro.getYaw();
                
            //keep bot straight during record
            driveRightXstick = yaw/20;
        }

        //to increment recording, make an index of when inputs are inputted
        executeCount++;
        
        //pushing values to smart dashbord
        SmartDashboard.putNumber("executeCount", executeCount);
        SmartDashboard.putNumber("getLeftX", m_driverController.getLeftX());
        SmartDashboard.putNumber("getRightX", m_driverController.getRightX());
        SmartDashboard.putNumber("getLeftY", m_driverController.getLeftY());

        SmartDashboard.putNumber("getYaw", m_robotDrive.getHeading());
        SmartDashboard.putNumber("getManipLeftY",manipulatorLeftYstick);

        /* AH slow down the robot
        if(buttonStartDriver == true){
            m_robotDrive.zeroHeading();
        }
        if(buttonR3Driver == true)
        {
            m_speedDivisor = 4;
            m_speedDivisorRotation = 4;
        }
        else if(buttonL3Driver == true)
        {
            m_speedDivisor = 2;
            m_speedDivisorRotation = 2;
        }
        else
        {
            m_speedDivisor = 1;
            m_speedDivisorRotation = 1;
        }
        */

        //Teleop, using the drive
        m_robotDrive.drive(
            //adds deadband (makes it so the controls aren't hyper sensitive)
            -MathUtil.applyDeadband( m_SlewY.calculate(driveLeftYstick/m_speedDivisor), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_SlewX.calculate(driveLeftXstick/m_speedDivisor), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(driveRightXstick/m_speedDivisorRotation, OIConstants.kDriveDeadband),
            true, false);
        m_robotDrive.periodic(); 


    
        //Teleop 

        
        if ((m_driveType != DriveType.Telop) && (executeCount >= 750))
        {
            if (m_driveType == DriveType.Record)
            {
            //save to file
              JoyReadWrite.writeObject(m_joy, m_filename);
            }

            //what does this do/why do we need it
            m_robotDrive.drive(
                -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
                true, false);
        }       
    } //end of execute

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
          //executeCount++;
          if ((m_driveType != DriveType.Telop) && (executeCount >= (m_seconds / 0.02)))
            {
               return true;
          }
          else
          {
            return false;
          }
          
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
