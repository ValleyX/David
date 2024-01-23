// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDown extends Command {
  private ClimberSubsystem m_ClimberSubsystem;

  public ClimberDown(ClimberSubsystem climbsub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = climbsub;
    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  
  //activate the climber

  
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.

  //check if encoders and angle are where they should be
  @Override
  public boolean isFinished() {
    return false;
  }
}
