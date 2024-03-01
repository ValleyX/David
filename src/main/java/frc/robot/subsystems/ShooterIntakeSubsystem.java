// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputIDs;
import frc.robot.Constants.RevCanIDs;
import java.util.function.BooleanSupplier;

public class ShooterIntakeSubsystem extends SubsystemBase {
  /** Creates a new ShooterIntakeSubsystem. */
  private final TalonSRX m_ShooterIntakeMotor; // Used to intake from floor intake motors to
  // feed shooter motors

  private final DigitalInput m_ShooterBeams;
  private BooleanSupplier m_shooterBeamsSupplier;

  public ShooterIntakeSubsystem() {

    m_ShooterIntakeMotor = new TalonSRX(RevCanIDs.kCAN_ShooterIntakeMotor);
    m_ShooterBeams = new DigitalInput(DigitalInputIDs.kDIG_ShootBeamBreak); // shooter beam break
    // addChild(getName(), m_ShooterBeams);
    m_shooterBeamsSupplier = () -> m_ShooterBeams.get();
    SmartDashboard.putBoolean("shooter Beam Break", m_ShooterBeams.get());
    m_ShooterIntakeMotor.setInverted(true);
    m_ShooterIntakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void MoveShooterIntake(double speed) {
    m_ShooterIntakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public BooleanSupplier
      IsNoteRecieved() { // checks if shooter beambreak is broken to see if we have a note
    return m_shooterBeamsSupplier;
  }

  public boolean
      IsNoteRecievedBool() { // checks if shooter beambreak is broken to see if we have a note
    return m_ShooterBeams.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("shooter Beam Break", m_ShooterBeams.get());
  }
}
