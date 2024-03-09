// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class OutTake extends Command {
  private IntakeSubsystem m_IntakeSubsystem;
  private ShooterIntakeSubsystem m_ShooterIntakeSubsystem;
  private PivotSubsystem m_PivotSubsystem;
  // private Joystick m_joystickManipulator;
  private int wait = 0;

  public OutTake(
      IntakeSubsystem intakesub, ShooterIntakeSubsystem shootersub, PivotSubsystem PivotSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakesub;
    m_ShooterIntakeSubsystem = shootersub;
    m_PivotSubsystem = PivotSub;
    // m_joystickManipulator = manipulate;
    addRequirements(m_IntakeSubsystem);
    addRequirements(m_ShooterIntakeSubsystem);
    addRequirements(m_PivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_IntakeSubsystem.out();
    m_ShooterIntakeSubsystem.MoveShooterIntake(-.4);
    wait = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
    m_ShooterIntakeSubsystem.MoveShooterIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
