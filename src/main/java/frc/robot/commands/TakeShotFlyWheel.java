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
  private double m_timeOut20Ms;
  private int waitCount;
  private double m_RPM;
  private boolean m_shootEngaged;
  private final double M_ERROR = 10; // in rpm
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 6000.0);
  private int m_timeOutCounter;

  public TakeShotFlyWheel(
      ShooterIntakeSubsystem shooterintakesubs, Flywheel flywheel, double RPM, int timeOutSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterIntakeSubsystem = shooterintakesubs;
    m_Flywheel = flywheel;
    waitCount = 0;
    if (RPM > 6700) { // safety so motor doesn't overload
      RPM = 6700;
    }
    m_RPM = RPM;
    m_shootEngaged = false;

    m_timeOut20Ms = timeOutSeconds / 0.02; // converts seconds to number of 20ms ticks

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
    m_timeOutCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_timeOutCounter++; // adds 1 to this every 20ms in this loop

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
  public boolean isFinished() { // TODO: ADD LED HEADS UP FOR WHEN SHOT FAILS DUE TO LOW RPM
    if ((waitCount > 20) || (m_timeOutCounter >= m_timeOut20Ms)) { // 15 is perfect!!!

      // m_Flywheel.runVolts(0);
      // m_ShooterIntakeSubsystem.MoveShooterIntake(0);
      return true;
    }
    return false;
  }
}
