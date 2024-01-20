// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAmpCommand extends Command {
  
  private ShooterSubsystem m_ShooterSubsystem;
  private Joystick m_joystickManipulator;

  public ShooterAmpCommand(ShooterSubsystem shootersubs, Joystick manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
   m_ShooterSubsystem = shootersubs;
   m_joystickManipulator = manipulate;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.MoveElevator(.5);//we can change speed values
    m_ShooterSubsystem.MovePivot(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_ShooterSubsystem.GetElevatorPosition() == 0){//TODO: replace 0 with value for height
      m_ShooterSubsystem.MoveElevator(0);
    }
    if(m_ShooterSubsystem.GetPivotPosition() == 0){//TODO: replace 0 with value for angle
      m_ShooterSubsystem.MovePivot(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ShooterSubsystem.GetElevatorPosition() == 0 && m_ShooterSubsystem.GetPivotPosition() == 0){//TODO: replace place holder values
      return true;
    }
    return false;
  }
}
