// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterReset extends Command {
  /** Creates a new ShooterReset. */
  private ShooterSubsystem m_ShooterSubsystem;

  private Joystick m_joystickManipulator;

  public ShooterReset(ShooterSubsystem shooterSubs, Joystick manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shooterSubs;
    m_joystickManipulator = manipulate;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.MoveElevator(-.5);
    m_ShooterSubsystem.MovePivot(-.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterSubsystem.IsElevatorDown()) {
      m_ShooterSubsystem.MoveElevator(0);
    }
    if (m_ShooterSubsystem.IsShooterAligned()) {
      m_ShooterSubsystem.MovePivot(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ShooterSubsystem.IsElevatorDown() && m_ShooterSubsystem.IsShooterAligned()) {
      return true;
    }
    return false;
  }
}
