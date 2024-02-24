// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotMoveToPosition extends Command {
  /** Creates a new PivotMoveToPosition. */
  private PivotSubsystem m_PivotSubsystem;

  private double m_PivotDegrees;

  public PivotMoveToPosition(PivotSubsystem pivotSubs, double pivotDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PivotSubsystem = pivotSubs;
    m_PivotDegrees = pivotDegrees;
    addRequirements(m_PivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PivotSubsystem.MovePivotTo(m_PivotDegrees);
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
    if (Math.abs(m_PivotSubsystem.pivotPos() - m_PivotDegrees) < 2) {
      return true;
    }

    return false;
  }
}
