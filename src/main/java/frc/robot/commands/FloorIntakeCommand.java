// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class FloorIntakeCommand extends Command {

  private IntakeSubsystem m_IntakeSubsystem;
  private ShooterIntakeSubsystem m_ShooterIntakeSubsystem;
  private PivotSubsystem m_PivotSubsystem;
  // private Joystick m_joystickManipulator;
  private int wait = 0;

  private PWM m_TowerBlinkin;
  private PWM m_BaseBlinkin;

  public FloorIntakeCommand(
      IntakeSubsystem intakesub, ShooterIntakeSubsystem shootersub, PivotSubsystem PivotSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakesub;
    m_ShooterIntakeSubsystem = shootersub;
    m_PivotSubsystem = PivotSub;
    // m_joystickManipulator = manipulate;
    addRequirements(m_IntakeSubsystem);
    addRequirements(m_ShooterIntakeSubsystem);
    addRequirements(m_PivotSubsystem);

    // m_TowerBlinkin = new PWM(PWMIDs.kPWM_TowerBlinkin);
    // m_BaseBlinkin = new PWM(PWMIDs.kPWM_BaseBlinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_IntakeSubsystem.in();
    m_ShooterIntakeSubsystem.MoveShooterIntake(.3);
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

    if (!m_ShooterIntakeSubsystem.IsNoteRecievedBool()) {

      // sets LEDs to gold when there is a ring in the thing
      // new BlinkinControl(m_BaseBlinkin, LightConstants.kGold);
      // new BlinkinControl(m_TowerBlinkin, LightConstants.kGold);

      return true;
    }

    // sets LEDs to gold when there is a ring in the thing
    // new BlinkinControl(m_BaseBlinkin, LightConstants.kBreathBlue);
    // new BlinkinControl(m_TowerBlinkin, LightConstants.kBreathBlue);

    return false;
  }
}
