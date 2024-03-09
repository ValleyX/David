// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorMoveToPosition;
import frc.robot.commands.FloorIntakeCommand;
import frc.robot.commands.OutTake;
import frc.robot.commands.PivotMoveToPosition;
import frc.robot.commands.TakeShotFlyWheel;
import frc.robot.commands.flywheelRev;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  // private final ShooterSubsystem shooterSub;
  private final Flywheel flywheel;
  private final ElevatorSubsystem elevatorSub;
  private final PivotSubsystem pivotSub;
  private final ShooterIntakeSubsystem shooterIntakeSub;
  private final IntakeSubsystem intakeSub;
  private boolean minorMode;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController manip = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 6000.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(3),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(0));
        // shooterSub = new ShooterSubsystem();
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        elevatorSub = new ElevatorSubsystem();
        pivotSub = new PivotSubsystem();
        shooterIntakeSub = new ShooterIntakeSubsystem();
        intakeSub = new IntakeSubsystem();
        minorMode = false;
        /*
              drive =
                  new Drive(
                      new GyroIOPigeon2(),
                      new ModuleIOTalonFX(0),
                      new ModuleIOTalonFX(1),
                      new ModuleIOTalonFX(2),
                      new ModuleIOTalonFX(3));
              flywheel = new Flywheel(new FlywheelIOTalonFX());
        */
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        /// shooterSub = new ShooterSubsystem();
        elevatorSub = new ElevatorSubsystem();
        pivotSub = new PivotSubsystem();
        shooterIntakeSub = new ShooterIntakeSubsystem();
        intakeSub = new IntakeSubsystem();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // shooterSub = new ShooterSubsystem();
        flywheel = new Flywheel(new FlywheelIO() {});
        elevatorSub = new ElevatorSubsystem();
        pivotSub = new PivotSubsystem();
        intakeSub = new IntakeSubsystem();
        shooterIntakeSub = new ShooterIntakeSubsystem();
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));

    NamedCommands.registerCommand("stop", Commands.runOnce(drive::stopWithX, drive));

    NamedCommands.registerCommand(
        "NonCentered Auto Shot",
        new PivotMoveToPosition(pivotSub, 45)
            .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 6000.0, 6)));

    NamedCommands.registerCommand(
        "Subwoofer Auto Shot",
        new PivotMoveToPosition(pivotSub, 58)
            .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 3500.0, 6)));

    NamedCommands.registerCommand(
        "Activate Intake",
        new PivotMoveToPosition(pivotSub, 60)
            .andThen(new FloorIntakeCommand(intakeSub, shooterIntakeSub, pivotSub)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // autoChooser.addOption("test", elevatorSub.getDefaultCommand());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    /*
            autoChooser.addOption(
            "Flywheel SysId (Quasistatic Forward)",
            flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Flywheel SysId (Quasistatic Reverse)",
            flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                    drive)
                .ignoringDisable(true));

    // MINOR MODE OH YEAHHH
    driver
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY() / 2,
                () -> -driver.getLeftX() / 2,
                () -> -driver.getRightX() / 2)
            // divinding by 4 so it goes quarter speed :D
            );

    // intake control: intake
    manip
        .leftTrigger()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 60)
                .andThen(new FloorIntakeCommand(intakeSub, shooterIntakeSub, pivotSub)));

    // intake control: intake cancel
    manip
        .rightTrigger()
        .whileTrue(
            new PivotMoveToPosition(pivotSub, 60)
                .andThen(new OutTake(intakeSub, shooterIntakeSub, pivotSub)));

    // shooter: auto not button button thingy tester ig :)
    manip
        .povUp()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 45)
                .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 6000.0, 6)));

    // shooter: speaker shot
    manip
        .a()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 58)
                .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 3500.0, 6)));

    // shooter: amp shot TODO: NEEDS TUNE + LOWER, MAKE ROUTINE
    manip
        .b()
        .onTrue(
            new PivotMoveToPosition(pivotSub, -55)
                .alongWith(new ElevatorMoveToPosition(elevatorSub, 9.5))
                .alongWith(new flywheelRev(flywheel, true, 3000))
                .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 3000, 6))
                .andThen(new ElevatorMoveToPosition(elevatorSub, 0))
                .andThen(new PivotMoveToPosition(pivotSub, 60)));

    // shooter: white line shot
    manip
        .y()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 40)
                .alongWith(new ElevatorMoveToPosition(elevatorSub, 15))
                .alongWith(new flywheelRev(flywheel, true, 5000))
                .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 5000, 6))
                .andThen(new ElevatorMoveToPosition(elevatorSub, 0))
                .andThen(new PivotMoveToPosition(pivotSub, 60)));

    // lift: reset lift
    manip
        .povLeft()
        .onTrue(
            new ElevatorMoveToPosition(elevatorSub, 0)
                .andThen(new PivotMoveToPosition(pivotSub, 60)));

    // intake: outtake
    manip
        .povDown()
        .whileTrue(
            new PivotMoveToPosition(pivotSub, 60)
                .andThen(new FloorIntakeCommand(intakeSub, shooterIntakeSub, pivotSub)));

    // flywheel: rev
    manip.rightBumper().onTrue(new flywheelRev(flywheel, true, 3500));

    // flywheel: cancel rev
    manip.leftBumper().onTrue(new flywheelRev(flywheel, false, 0));

    // manip.x().onTrue(new PivotMoveToPosition(pivotSub, 37));
    // manip.b().onTrue(new PivotMoveToPosition(pivotSub, 0));
    // manip.a().onTrue(new PivotMoveToPosition(pivotSub, -10));
    // manip
    //     .y()
    //     .onTrue(
    //         new PivotMoveToPosition(pivotSub, 60)
    //             .andThen(new FloorIntakeCommand(intakeSub, shooterIntakeSub, pivotSub)));

    // manip.leftBumper().onTrue(new ElevatorMoveToPosition(elevatorSub, 5));
    // manip.rightBumper().onTrue(new ElevatorMoveToPosition(elevatorSub, 10));
    // manip.povUp().onTrue(new ElevatorMoveToPosition(elevatorSub, 0));
    // manip
    //     .povDown()
    //     .onTrue(
    //         new ElevatorMoveToPosition(elevatorSub, 16)
    //             .andThen(new PivotMoveToPosition(pivotSub, 60)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return elevatorSub.getCurrentCommand();
  }

  public void EnterTelop() {
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Math.PI)));
  }
}
