// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class HumanPlayerIntakeCommand extends Command {
  /** Creates a new HumanPlayerIntake. */
  private ShooterSubsystem m_ShooterSubsystem;
  private Joystick m_joystickManipulator;

  enum NoteState {
    WAITING_FOR_NOTE,
    NOTE_RECIEVED,
    NOTE_PAST_BEAM_BREAK
  }

  private NoteState noteState;

  

  public HumanPlayerIntakeCommand(ShooterSubsystem shootersub, Joystick manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shootersub;
    m_joystickManipulator = manipulate;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.ExtakeOut();
    noteState = NoteState.WAITING_FOR_NOTE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterSubsystem.IsNoteRecieved() && noteState == NoteState.WAITING_FOR_NOTE) {
      noteState = NoteState.NOTE_RECIEVED;
    }
    if(!m_ShooterSubsystem.IsNoteRecieved() && noteState == NoteState.NOTE_RECIEVED){
      noteState = NoteState.NOTE_PAST_BEAM_BREAK;
      m_ShooterSubsystem.IntakeIn();
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ShooterSubsystem.IsNoteRecieved() && noteState == NoteState.NOTE_PAST_BEAM_BREAK){
      return true;
    }
    return false;
  }
}
