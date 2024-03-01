// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TakeShotFlyWheel extends Command {
  /** Creates a new TakeShot. */
  private ShooterIntakeSubsystem m_ShooterIntakeSubsystem;

  private Flywheel m_Flywheel;
  private int waitCount;
  private double m_RPM;
  private boolean m_shootEngaged;
  private final double M_ERROR = 10; // in rpm
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 6000.0);

  public TakeShotFlyWheel(ShooterIntakeSubsystem shooterintakesubs, Flywheel flywheel, Double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterIntakeSubsystem = shooterintakesubs;
    m_Flywheel = flywheel;
    waitCount = 0;
    m_RPM = RPM;
    m_shootEngaged = false;

    addRequirements(m_ShooterIntakeSubsystem, m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_ShooterIntakeSubsystem.MoveShooterIntake(.5);
    waitCount = 0;
    // m_Flywheel.runVelocity(flywheelSpeedInput.get());
    // m_Flywheel.runVolts(0.9);
    m_Flywheel.runVelocity(m_RPM);
    m_shootEngaged = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_Flywheel.getVelocityRPM() > m_RPM) {
      m_ShooterIntakeSubsystem.MoveShooterIntake(1);
      m_shootEngaged = true;
    }
    if (m_shootEngaged) {
      waitCount++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterIntakeSubsystem.MoveShooterIntake(0);
    m_Flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (waitCount > 100) {

      // m_Flywheel.runVolts(0);
      // m_ShooterIntakeSubsystem.MoveShooterIntake(0);
      return true;
    }
    return false;
  }
}
