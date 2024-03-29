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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.LightConstants;
import frc.robot.commands.BlinkinControl;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ClimberMoveToPosition;
import frc.robot.commands.ClimberReset;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorMoveToPosition;
import frc.robot.commands.FloorIntakeCommand;
import frc.robot.commands.OutTake;
import frc.robot.commands.PivotMoveToPosition;
import frc.robot.commands.TakeShotFlyWheel;
import frc.robot.commands.flywheelRev;
import frc.robot.commands.flywheelRevUp;
import frc.robot.subsystems.BlinkinSub;
import frc.robot.subsystems.ClimberSubsystem;
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

  // BLINKINS :D
  private BlinkinSub m_TowerBlinkin;
  private BlinkinSub m_BaseBlinkin;

  // Subsystems
  private final Drive drive;
  // private final ShooterSubsystem shooterSub;
  private final Flywheel flywheel;
  private final ElevatorSubsystem elevatorSub;
  private final PivotSubsystem pivotSub;
  private final ShooterIntakeSubsystem shooterIntakeSub;
  private final IntakeSubsystem intakeSub;
  private final ClimberSubsystem climberSub;
  private boolean minorMode;

  private int counter = 0;

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
                new ModuleIOSparkFlexFx(3),
                new ModuleIOSparkFlexFx(2),
                new ModuleIOSparkFlexFx(1),
                new ModuleIOSparkFlexFx(0)
                // new ModuleIOSparkMax(0)
                );
        // shooterSub = new ShooterSubsystem();
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        elevatorSub = new ElevatorSubsystem();
        pivotSub = new PivotSubsystem();
        shooterIntakeSub = new ShooterIntakeSubsystem();
        intakeSub = new IntakeSubsystem();
        climberSub = new ClimberSubsystem();
        minorMode = false;
        m_TowerBlinkin = new BlinkinSub(Constants.PWMIDs.kPWM_TowerBlinkin);
        m_BaseBlinkin = new BlinkinSub(Constants.PWMIDs.kPWM_BaseBlinkin);
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
        climberSub = new ClimberSubsystem();
        m_TowerBlinkin = new BlinkinSub(Constants.PWMIDs.kPWM_TowerBlinkin);
        m_BaseBlinkin = new BlinkinSub(Constants.PWMIDs.kPWM_BaseBlinkin);
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
        climberSub = new ClimberSubsystem();
        m_TowerBlinkin = new BlinkinSub(Constants.PWMIDs.kPWM_TowerBlinkin);
        m_BaseBlinkin = new BlinkinSub(Constants.PWMIDs.kPWM_BaseBlinkin);
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand("Run Flywheel", new flywheelRev(flywheel, true, 4500));

    NamedCommands.registerCommand("stop", Commands.runOnce(drive::stopWithX, drive));

    NamedCommands.registerCommand(
        "NonCentered Auto Shot",
        new PivotMoveToPosition(pivotSub, 50)
            .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 5800.0, 6, m_TowerBlinkin)));

    NamedCommands.registerCommand(
        "Subwoofer Auto Shot",
        new PivotMoveToPosition(pivotSub, 59)
            .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 4000.0, 6, m_TowerBlinkin)));

    NamedCommands.registerCommand(
        "Subwoofer Auto Shot Back a Little",
        new PivotMoveToPosition(pivotSub, 57)
            .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 4000.0, 6, m_TowerBlinkin)));

    NamedCommands.registerCommand(
        "Activate Intake",
        new PivotMoveToPosition(pivotSub, 60)
            .andThen(
                new FloorIntakeCommand(
                        intakeSub, shooterIntakeSub, pivotSub, m_TowerBlinkin, m_BaseBlinkin)
                    .withTimeout(3.5)));

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
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY() / 2.5,
                () -> -driver.getLeftX() / 2.5,
                () -> -driver.getRightX() / 2.5)
            // divinding by 4 so it goes quarter speed :D
            );

    // blinkin Tests
    driver
        .y()
        .onTrue(new BlinkinControl(m_BaseBlinkin, LightConstants.kGold))
        .onTrue(new BlinkinControl(m_TowerBlinkin, LightConstants.kGold))
        .onFalse(new BlinkinControl(m_BaseBlinkin, LightConstants.kBreathBlue))
        .onFalse(new BlinkinControl(m_TowerBlinkin, LightConstants.kBreathBlue));

    // intake control: intake
    manip
        .leftTrigger()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 61)
                .andThen(
                    new FloorIntakeCommand(
                        intakeSub, shooterIntakeSub, pivotSub, m_TowerBlinkin, m_BaseBlinkin)));

    // intake control: outtake
    manip
        .rightTrigger()
        .whileTrue(
            new PivotMoveToPosition(pivotSub, 61)
                .andThen(new OutTake(intakeSub, shooterIntakeSub, pivotSub)));

    // shooter: speaker shot
    manip
        .a()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 58)
                .andThen(
                    new TakeShotFlyWheel(shooterIntakeSub, flywheel, 4000.0, 6, m_TowerBlinkin)));
    // 3500

    // shooter: amp shot
    manip
        .b()
        .onTrue(
            new PivotMoveToPosition(pivotSub, -55)
                .alongWith(new ElevatorMoveToPosition(elevatorSub, 9.5))
                .alongWith(new flywheelRev(flywheel, true, 3000))
                .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 3000, 6, m_TowerBlinkin))
                .andThen(new ElevatorMoveToPosition(elevatorSub, 0))
                .andThen(new PivotMoveToPosition(pivotSub, 60)));

    // shooter: white line shot
    manip
        .y()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 35)
                .andThen(new ElevatorMoveToPosition(elevatorSub, 15))
                .alongWith(new flywheelRev(flywheel, true, 5000))
                .andThen(new TakeShotFlyWheel(shooterIntakeSub, flywheel, 5000, 6, m_TowerBlinkin))
                .andThen(new ElevatorMoveToPosition(elevatorSub, 0))
                .andThen(new PivotMoveToPosition(pivotSub, 60)));

    // lift: reset lift
    manip
        .povLeft()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 40)
                .andThen(
                    new ElevatorMoveToPosition(elevatorSub, 0)
                        .andThen(new PivotMoveToPosition(pivotSub, 60))));

    // intake: cancel/manual
    manip
        .povRight()
        .whileTrue(
            new PivotMoveToPosition(pivotSub, 60)
                .andThen(
                    new FloorIntakeCommand(
                        intakeSub, shooterIntakeSub, pivotSub, m_TowerBlinkin, m_BaseBlinkin)));

    // flywheel: rev
    manip.rightBumper().onTrue(new flywheelRevUp(flywheel, true, 5500));
    // 3500

    // flywheel: cancel rev
    manip.leftBumper().onTrue(new flywheelRev(flywheel, false, 0));

    // runs lift off stick
    climberSub.setDefaultCommand(
        ClimberCommand.joystickControl(climberSub, () -> -manip.getRightY()));

    // moves climber to set pos in degrees
    manip.povUp().onTrue(new ClimberMoveToPosition(climberSub, 20));

    // reset climber to 0
    manip.povDown().onTrue(new ClimberReset(climberSub));

    manip
        .x()
        .onTrue(
            new PivotMoveToPosition(pivotSub, 40)
                .andThen(
                    new ElevatorMoveToPosition(elevatorSub, 15)
                        .andThen(new PivotMoveToPosition(pivotSub, 60))));
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

  // sets wheels to coast
  public void setCoast() {
    drive.runVelocity(new ChassisSpeeds(0.01, 0, 0));
    drive.stop();
    drive.setDriveBrakeMode(false);
  }

  public void teleopCounter() {
    counter++;
    SmartDashboard.putNumber("TeleOp Counter: ", counter);
    if (shooterIntakeSub.IsNoteRecievedBool()
        && intakeSub.getFloorBeans()
        && !intakeSub.isFloorIntakeOn()) {
      m_TowerBlinkin.setColor(LightConstants.kBreathBlue);
    }
  }
}
