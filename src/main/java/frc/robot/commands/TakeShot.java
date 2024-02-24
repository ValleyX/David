// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;

public class TakeShot extends Command {
  /** Creates a new TakeShot. */
  private ShooterIntakeSubsystem m_ShooterIntakeSubsystem;

  private Flywheel m_Flywheel;
  private int waitCount;

  public TakeShot(ShooterIntakeSubsystem shooterintakesubs, Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterIntakeSubsystem = shooterintakesubs;
    m_Flywheel = flywheel;
    waitCount = 0;

    addRequirements(m_ShooterIntakeSubsystem, m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterIntakeSubsystem.MoveShooterIntake(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    waitCount++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (waitCount > 5) {
      m_ShooterIntakeSubsystem.MoveShooterIntake(0);
      m_Flywheel.stop();
      return true;
    }
    return false;
  }
}
