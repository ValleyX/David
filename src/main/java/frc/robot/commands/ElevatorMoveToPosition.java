// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveToPosition extends Command {
  /** Creates a new ElevatorMoveToPosition. */
  private ElevatorSubsystem m_ElevatorSubsystem;

  private double m_ElevatorInches;

  public ElevatorMoveToPosition(ElevatorSubsystem elevatorSubs, double elevatorInch) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElevatorSubsystem = elevatorSubs;
    m_ElevatorInches = elevatorInch;

    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.MoveElevatorTo(m_ElevatorInches);
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
    if (Math.abs(m_ElevatorSubsystem.elevatorPos() - m_ElevatorInches) < 2) {
      return true;
    }
    return false;
  }
}
