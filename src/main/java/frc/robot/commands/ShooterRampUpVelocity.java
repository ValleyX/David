// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

public class ShooterRampUpVelocity extends Command {
  /** Creates a new ShooterRampUpVelocity. */
  private Flywheel m_Flywheel;

  private double m_VelocityRPM;
  private final double VELOCITY_DEADBAND = 100;

  public ShooterRampUpVelocity(Flywheel flywheel, double veloRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Flywheel = flywheel;
    m_VelocityRPM = veloRPM;
    addRequirements(m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheel.runVelocity(m_VelocityRPM);
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
    if (m_Flywheel.getVelocityRPM() > (m_VelocityRPM - VELOCITY_DEADBAND)) {
      return true;
    }
    return false;
  }
}
