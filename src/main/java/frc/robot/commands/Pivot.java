package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PivotSubsystem;
import java.util.function.DoubleSupplier;

public class Pivot {

  public static Command joystickControl(PivotSubsystem pivotSub, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          pivotSub.MovePivot((ySupplier.getAsDouble() / 8)); // the divide by 8 for testing
        },
        pivotSub);
  }
}
