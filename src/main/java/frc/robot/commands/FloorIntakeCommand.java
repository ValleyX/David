// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FloorIntakeCommand extends Command {

  private IntakeSubsystem m_IntakeSubsystem;
  private ShooterSubsystem m_ShooterSubsystem; 
  private Joystick m_joystickManipulator;

  public FloorIntakeCommand(IntakeSubsystem intakesub, ShooterSubsystem shootersub, Joystick manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakesub;
    m_ShooterSubsystem = shootersub;
    m_joystickManipulator = manipulate;
    addRequirements(m_IntakeSubsystem);
    addRequirements(m_ShooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ShooterSubsystem.IsShooterAligned()){
      m_IntakeSubsystem.IntakeIn();
      m_ShooterSubsystem.IntakeIn();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ShooterSubsystem.IsNoteRecieved()){
      m_IntakeSubsystem.IntakeStop();
      m_ShooterSubsystem.IntakeStop();
      return true;
    }
    return false;
  }
}
