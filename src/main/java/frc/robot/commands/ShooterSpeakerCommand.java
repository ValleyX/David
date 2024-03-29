// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSpeakerCommand
    extends Command { // moves shooter to subwoofer position for shooting MIght not need if we shoot
  // rom starting position

  private ShooterSubsystem m_ShooterSubsystem;
  private Joystick m_joystickManipulator;

  public ShooterSpeakerCommand(ShooterSubsystem shootersubs, Joystick manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shootersubs;
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
