// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class Climber {

  public static Command joystickControl(ClimberSubsystem climberSub, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          climberSub.moveClimbers((ySupplier.getAsDouble() / 8)); // the divide by 8 for testing
        },
        climberSub);
  }
}
