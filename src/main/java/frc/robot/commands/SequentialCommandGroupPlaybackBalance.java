// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;
import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.TelopCommand.DriveType;
import frc.robot.subsystems.DriveSubsystem;


public class SequentialCommandGroupPlaybackBalance extends SequentialCommandGroup {

   // private final DriveSubsystem m_robotDrive;
    //private final LiftSubsystem m_LiftSubsystem;

  

    public SequentialCommandGroupPlaybackBalance(TelopCommand.DriveType driveType, int seconds, String filename, DriveSubsystem driveTrain )
    {

   
    addCommands(
     

        new TelopCommand(driveType, seconds, filename, driveTrain)
        //put banancing command call here
        );
    }



    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
