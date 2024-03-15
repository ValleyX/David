// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

public class flywheelRevUp extends Command {
  /** Creates a new flywheelRev. */
  private Flywheel m_flywheel;

  private boolean m_enable;
  private int m_targetRPM;

  public flywheelRevUp(Flywheel flywheel, boolean enable, int targetRPM) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_flywheel = flywheel;
    m_enable = enable;
    m_targetRPM = targetRPM;

    addRequirements(flywheel);
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

    m_flywheel.runVelocity(m_targetRPM);

    return false;
  }
} // MAKES THE FLYWHEEL HOPEFULLY HOLD A CONSTANT SPEED AT A GIVEN RPM
