// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberReset extends Command {
  /** Creates a new ClimberNotfollowingCommand. */
  private ClimberSubsystem m_ClimberSubsystem;

  public ClimberReset(ClimberSubsystem climberSubs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = climberSubs;
    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_ClimberSubsystem.initUnfollow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimberSubsystem.moveBothClimbers(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ClimberSubsystem.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ClimberSubsystem.IsDownLeft() && m_ClimberSubsystem.IsDownRight()) {
      return true;
    } else {
      return false;
    }
  }
}
