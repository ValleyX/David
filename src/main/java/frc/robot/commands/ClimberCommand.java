package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class ClimberCommand extends Command {

  public ClimberCommand(ClimberSubsystem climberSub, int i) {
    // TODO Auto-generated constructor stub
  }

  public static Command joystickControl(ClimberSubsystem climberSub, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          climberSub.moveClimbers(ySupplier.getAsDouble()); // the divide by 4 for testing
        },
        climberSub);
  }
}
