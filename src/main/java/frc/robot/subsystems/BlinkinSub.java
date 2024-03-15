// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinSub extends SubsystemBase {

  private PWM m_PWM;

  /** Creates a new BlinkinSub. */
  public BlinkinSub(int BlinkinPort) {
    m_PWM = new PWM(BlinkinPort);
  }

  // function for setting the color
  public void setColor(double color) {
    m_PWM.setSpeed(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
