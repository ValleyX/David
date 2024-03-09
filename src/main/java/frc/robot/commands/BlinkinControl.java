// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;

public class BlinkinControl extends Command {
  /** Creates a new BlinkinControl. */
  private PWM m_blinkin;

  private double m_color;

  public BlinkinControl(PWM blinkin, double color) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_blinkin = blinkin; // saves off blinkin id
    m_color = color; // saves off color id
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_blinkin.setSpeed(m_color);
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
    return true;
  }
}
