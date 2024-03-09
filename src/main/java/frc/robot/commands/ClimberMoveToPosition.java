// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMoveToPosition extends Command {
  /** Creates a new PivotMoveToPosition. */
  private ClimberSubsystem m_ClimberSubsystem;

  private double m_ClimberDegrees;

  public ClimberMoveToPosition(ClimberSubsystem climberSub, double climberDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = climberSub;
    m_ClimberDegrees = climberDegrees;
    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberSubsystem.moveClimberTo(m_ClimberDegrees);
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
    if (Math.abs(
            ((m_ClimberSubsystem.climberPos() / Constants.Pivot.RotationsPerDegree))
                - m_ClimberDegrees)
        < 1) {
      return true;
    }

    return false;
  }
}
