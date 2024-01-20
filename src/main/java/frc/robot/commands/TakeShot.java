// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class TakeShot extends Command {
  /** Creates a new TakeShot. */
  private ShooterSubsystem m_ShooterSubsystem;
  private Joystick m_joystickManipulator;
  private boolean IsShooterEngaged;
  private int waitCount;
  public TakeShot(ShooterSubsystem shootersubs, Joystick manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shootersubs;
   m_joystickManipulator = manipulate;
   IsShooterEngaged = false;
   waitCount = 0;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.MoveShooter(.5);//TODO: write constants for shoter subsystem to use here
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterSubsystem.GetRPM() > 0){
      m_ShooterSubsystem.IntakeIn();
      IsShooterEngaged = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (IsShooterEngaged){
      waitCount++;
    }
    if (waitCount > 2) {//waits 40 milliseconds after the thing is true
      return true;
    }
    return false;
  }
}
