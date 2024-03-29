// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TakeShot extends Command {
  /** Creates a new TakeShot. */
  private ShooterIntakeSubsystem m_ShooterIntakeSubsystem;

  private Flywheel m_Flywheel;
  private int waitCount;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 6000.0);

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
    // m_ShooterIntakeSubsystem.MoveShooterIntake(.5);
    waitCount = 0;
    // m_Flywheel.runVelocity(flywheelSpeedInput.get());
    m_Flywheel.runVolts(0.9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    waitCount++;
    if (waitCount == 250) {
      m_ShooterIntakeSubsystem.MoveShooterIntake(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Flywheel.runVolts(0);
    m_ShooterIntakeSubsystem.MoveShooterIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (waitCount > 300) {
      // m_ShooterIntakeSubsystem.MoveShooterIntake(0);
      // m_Flywheel.stop();
      // m_Flywheel.runVolts(0);
      // m_ShooterIntakeSubsystem.MoveShooterIntake(0);
      return true;
    }
    return false;
  }
}
