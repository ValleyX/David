package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorCommand {

  public static Command joystickControl(ShooterSubsystem shooterSub, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          shooterSub.MoveElevator(ySupplier.getAsDouble() / 4); // the divide by 4 for testing
        },
        shooterSub);
  }
}
