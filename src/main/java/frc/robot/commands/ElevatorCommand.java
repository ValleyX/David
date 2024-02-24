package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorCommand extends Command {

  public static Command joystickControl(ElevatorSubsystem elevatorSub, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          elevatorSub.MoveElevator(ySupplier.getAsDouble() / 4); // the divide by 4 for testing
        },
        elevatorSub);
  }
}
