package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class Pivot {

  public static Command joystickControl(ShooterSubsystem shooterSub, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          shooterSub.MovePivot((ySupplier.getAsDouble() / 8)); // the divide by 8 for testing
        },
        shooterSub);
  }
}
