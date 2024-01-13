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

  public HumanPlayerIntakeCommand(ShooterSubsystem shootersub, Joystick manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shootersub;
    m_joystickManipulator = manipulate;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
