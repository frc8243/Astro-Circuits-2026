// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  // private ShooterSubsystem m_shooter = new ShooterSubsystem();
  // private IntakeSubsystem m_intake = new IntakeSubsystem();

  private SwerveSubsystem drivebase;
  private String serialNum = System.getenv("serialnum");
  private SwerveInputStream driveAngularVelocity;
  private SwerveInputStream driveDirectAngle;
  private SwerveInputStream driveRobotOriented;
  private SwerveInputStream driveAngularVelocityKeyboard;
  private SwerveInputStream driveDirectAngleKeyboard;
  private static SendableChooser<Command> autoChooser;

  private static final Pose2d AUTO_START_POSE = new Pose2d(2, 7, Rotation2d.fromDegrees(0));

  private static final Pose2d STRAIGHT_POSE = new Pose2d(7, 5, Rotation2d.fromDegrees(0));

  private ArmSubsystem arm = new ArmSubsystem();

  void createSwerveInputStreams() {
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY() * -1,
        () -> driverXbox.getLeftX() * -1)
        .withControllerRotationAxis(driverXbox::getRightX)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
        driverXbox::getRightY)
        .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
        .allianceRelativeControl(false);

    driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX())
        .withControllerRotationAxis(() -> driverXbox.getRawAxis(
            2))
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);
    // Derive the heading axis with math!
    driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(() -> Math.sin(
            driverXbox.getRawAxis(
                2) *
                Math.PI)
            *
            (Math.PI *
                2),
            () -> Math.cos(
                driverXbox.getRawAxis(
                    2) *
                    Math.PI)
                *
                (Math.PI *
                    2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees(
            0));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    setupAuton();
    // the serial number to the alphabot is 031823E8
    // the serial number to the 2025 is 034159F4
    // 031823e8
    System.out.println("serialnum is " + serialNum);

    if (Robot.isSimulation()) {
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve/alphabot"));

    } else if (serialNum.equals("034159f4")) {
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve/2025"));
    } else if (serialNum.equals("031823e8"))
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve/alphabot"));
    createSwerveInputStreams();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // m_shooter.setDefaultCommand(m_shooter.setDutyCycle(0));
    // m_intake.setDefaultCommand(m_intake.stop());
    // fuelSubsystem.setDefaultCommand(fuelSubsystem.stop());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
    arm.setDefaultCommand(arm.armCmd(0));

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      driverXbox.povDown().whileTrue(arm.setAngle(Degrees.of(0)));
      driverXbox.povUp().whileTrue(arm.setAngle(Degrees.of(90)));

      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.start().whileTrue(Commands.none());
    driverXbox.back().whileTrue(Commands.none());
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
    // driverXbox.rightBumper().onTrue(Commands.none());
    /*
     * driverXbox.y().whileTrue(m_shooter.setDutyCycle(0.5));
     * driverXbox.b().whileTrue(m_shooter.setDutyCycle(-0.5));
     * driverXbox.leftBumper().whileTrue(m_intake.out(0.8));
     * driverXbox.rightBumper().whileTrue(m_intake.in(0.8));
     */

    // driverXbox.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // .toggleOnFalse(fuelSubsystem.stop());
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    // driverXbox.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake

    // driverXbox.a().whileTrue(new Eject(fuelSubsystem));
    // fuelSubsystem.setDefaultCommand(fuelSubsystem.run(()->fuelSubsystem.stop()));

    driverXbox.povUp()
        .onTrue(Commands.runOnce(
            () -> drivebase.resetOdometry(AUTO_START_POSE)));

    driverXbox.povRight()
        .onTrue(drivebase.driveToPose(STRAIGHT_POSE));

  }

  private void setupAuton() {
    autoChooser = new SendableChooser<>();
    Command driveStraight = Commands.sequence(
        Commands.runOnce(
            () -> drivebase.resetOdometry(AUTO_START_POSE)),
        drivebase.driveToPose(STRAIGHT_POSE))

    ;
    autoChooser.addOption("drivestraight", driveStraight);
    autoChooser.setDefaultOption("donothing", Commands.none());
    SmartDashboard.putData("Autos/Selector", autoChooser);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
