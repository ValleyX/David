package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class PivotandElevator {

  public static Command joystickControl(
      ShooterSubsystem shooterSub, DoubleSupplier yleft, DoubleSupplier yRight) {

    return Commands.run(
        () -> {
          shooterSub.MovePivotAndElevator(
              yleft.getAsDouble() / 4, yRight.getAsDouble() / 4); // the divide by 8,4 for testing
        },
        shooterSub);
  }
}
