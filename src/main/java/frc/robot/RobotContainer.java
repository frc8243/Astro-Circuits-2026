// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.WristAngle;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);
    // The robot's subsystems and commands are defined here...
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ArmSubsystem arm = new ArmSubsystem();
    private SwerveSubsystem drivebase;
    private String serialNum = System.getenv("serialnum");
    private SwerveInputStream driveAngularVelocity;
    private SwerveInputStream driveDirectAngle;
    private SwerveInputStream driveRobotOriented;
    private SwerveInputStream driveAngularVelocityKeyboard;
    private SwerveInputStream driveDirectAngleKeyboard;
    private static SendableChooser<Command> autoChooser;

    // private ArmSubsystem arm = new ArmSubsystem();

    private ShooterSubsystem shooter = new ShooterSubsystem();
    private HopperSubsystem hopper = new HopperSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();

    void createSwerveInputStreams() {
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
         * velocity.
         */
        driveAngularVelocity =
                SwerveInputStream.of(
                                drivebase.getSwerveDrive(),
                                () -> driverXbox.getLeftY() * -1,
                                () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(() -> -driverXbox.getRightX())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative input
         * stream.
         */
        driveDirectAngle =
                driveAngularVelocity
                        .copy()
                        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative input
         * stream.
         */
        driveRobotOriented =
                driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

        driveAngularVelocityKeyboard =
                SwerveInputStream.of(
                                drivebase.getSwerveDrive(),
                                () -> -driverXbox.getLeftY(),
                                () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        driveDirectAngleKeyboard =
                driveAngularVelocityKeyboard
                        .copy()
                        .withControllerHeadingAxis(
                                () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(0));
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // the serial number to the alphabot is 031823E8
        // the serial number to the 2025 is 034159F4
        // 031823e8
        System.out.println("serialnum is " + serialNum);

        if (Robot.isSimulation()) {
            drivebase =
                    new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/2026"));

        } else if (serialNum.equals("034159f4")) {
            drivebase =
                    new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/2026"));
        } else if (serialNum.equals("031823e8"))
            drivebase =
                    new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/2026"));
        else {
            drivebase =
                    new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/2026"));
        }
        createSwerveInputStreams();
        // Configure the trigger bindings
        // hopper.setDefaultCommand(hopper.out(0));
        // shooter.setDefaultCommand(shooter.defaultBehaviorCommand());
        arm.setDefaultCommand(
                new RunCommand(
                        () ->
                                arm.manualControl(
                                        -MathUtil.applyDeadband(
                                                operatorXbox.getLeftY(),
                                                OIConstants.kDriveDeadband)),
                        arm));

        // indexer.setDefaultCommand(indexer.out(0));
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        setupAuton();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        Command driveFieldOrientedAngularVelocity =
                drivebase.driveFieldOriented(driveAngularVelocity);

        Command driveFieldOrientedDirectAngleKeyboard =
                drivebase.driveFieldOriented(driveDirectAngleKeyboard);

        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        }

        if (Robot.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));

            driveDirectAngleKeyboard.driveToPose(
                    () -> target,
                    new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
                    new ProfiledPIDController(
                            5,
                            0,
                            0,
                            new Constraints(
                                    Units.degreesToRadians(360), Units.degreesToRadians(180))));
            driverXbox
                    .start()
                    .onTrue(
                            Commands.runOnce(
                                    () ->
                                            drivebase.resetOdometry(
                                                    new Pose2d(3, 3, new Rotation2d()))));
            driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            driverXbox
                    .button(2)
                    .whileTrue(
                            Commands.runEnd(
                                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
        }

        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

        operatorXbox.povLeft().whileTrue(arm.goToWristAngleCommand(WristAngle.DEPLOY));
        operatorXbox.povRight().whileTrue(arm.goToWristAngleCommand(WristAngle.STOW));
        operatorXbox.back().whileTrue(Commands.none());
        //        operatorXbox.a().whileTrue(shooter.out(-0.8)).whileFalse(shooter.in(0.0));
        // operatorXbox
        //         .a()
        //         .whileTrue(
        //                 Commands.startEnd(
        //                         () -> shooter.setVelocity(1000.0),
        //                         () -> shooter.setVelocity(0.0),
        //                         shooter));

        operatorXbox
                .a()
                .whileTrue(
                        shooter.spinToRPM(3000) // spin up
                                .until(() -> shooter.atSpeed(3000, 100))
                                .andThen(indexer.out(0.8).alongWith(shooter.spinToRPM(3000))));
        // operatorXbox.a().whileTrue(indexer.out(0.8).alongWith(shooter.setShooterSpeed(-0.8)));
        operatorXbox.x().whileTrue(hopper.out(-0.4)).whileFalse(hopper.in(0.0));
        // operatorXbox.y().whileTrue(indexer.out(0.8)).whileFalse(indexer.in(0.0));

        operatorXbox.b().whileTrue(intake.in(-1.0)).whileFalse(intake.in(0.0));
    }

    private static final Pose2d RIGHT_AUTO_START_POSE =
            new Pose2d(4, 0.7, Rotation2d.fromDegrees(0));
    private static final Pose2d RIGHT_AUTO_RETURN_POSE =
            new Pose2d(6, 0.7, Rotation2d.fromDegrees(0));
    private static final Pose2d STRAIGHT_POSE = new Pose2d(7.45, 0.7, Rotation2d.fromDegrees(90));
    private static final Pose2d NEUTRAL_ZONE_POSE2D =
            new Pose2d(7.7, 4, Rotation2d.fromDegrees(90));
    private static final Pose2d RIGHT_SHOOT_POSE_ = new Pose2d(3, 0.5, Rotation2d.fromDegrees(0));
    private static final Pose2d OUTPOST_TRENCH_SHOOT_POSE =
            new Pose2d(3, .7, Rotation2d.fromDegrees(60));
    private static final Pose2d DEPOT_TRENCH_SHOOT_POSE =
            new Pose2d(3, 7.5, Rotation2d.fromDegrees(-60));
    private static final Pose2d LEFT_AUTO_START_POSE =
            new Pose2d(4, 7.5, Rotation2d.fromDegrees(270));
    private static final Pose2d LEFT_AUTO_RETURN_POSE =
            new Pose2d(6, 7.5, Rotation2d.fromDegrees(270));

    private static final Pose2d FRONT_POSE = new Pose2d(7.5, 7, Rotation2d.fromDegrees(270));
    private static final Pose2d MIDDLE_ZONE_POSE2D =
            new Pose2d(7.7, 5, Rotation2d.fromDegrees(270));

    private static final Pose2d AUTO_START_POSE = new Pose2d(4, 1, Rotation2d.fromDegrees(0));

    private static final Pose2d MIDDLE_AUTO_START_POSE =
            new Pose2d(3, 4, Rotation2d.fromDegrees(0));
    private static final Pose2d MIDDLE_SHOOT_POSE = new Pose2d(3, 4, Rotation2d.fromDegrees(0));
    private static final Pose2d DEPOT_POSE = new Pose2d(.5, 7.5, Rotation2d.fromDegrees(90));
    private static final Pose2d DEPOT_PRE_POSE = new Pose2d(.5, 7, Rotation2d.fromDegrees(90));
    private static final Pose2d DEPOT_COLLECT_POSE =
            new Pose2d(.5, 5.5, Rotation2d.fromDegrees(360));
    private static final Pose2d OUTPOST_ZONE_POSE2D =
            new Pose2d(0.5, 0.716, Rotation2d.fromDegrees(180));

    private void setupAuton() {
        autoChooser = new SendableChooser<>();

        Command driveStraight =
                Commands.sequence(
                        Commands.runOnce(
                                () -> drivebase.resetOdometryDeferredFlip(AUTO_START_POSE)),
                        drivebase.driveToPoseDeferredWithFlip(STRAIGHT_POSE));

        Command outpostNeutralZone =
                Commands.sequence(
                        Commands.runOnce(
                                () -> drivebase.resetOdometryDeferredFlip(RIGHT_AUTO_START_POSE)),
                        drivebase.driveToPoseDeferredWithFlip(STRAIGHT_POSE),
                        // arm.goToWristAngleCommand(WristAngle.DEPLOY),
                        // intake.in(-1),
                        drivebase.driveToPoseDeferredWithFlip(NEUTRAL_ZONE_POSE2D),
                        drivebase.driveToPoseDeferredWithFlip(RIGHT_AUTO_RETURN_POSE),
                        drivebase.driveToPoseDeferredWithFlip(OUTPOST_TRENCH_SHOOT_POSE)
                        // drivebase.driveToPose(MIDDLE_SHOOT_POSE)
                        // intake.in(0.0),
                        // shooter.spinToRPM(3000),
                        // Commands.waitUntil(() -> shooter.atSpeed(3000, 100)),
                        // indexer.out(0.8)
                        );

        Command depotNeutralZone =
                Commands.sequence(
                        Commands.runOnce(
                                () -> drivebase.resetOdometryDeferredFlip(LEFT_AUTO_START_POSE)),
                        // drivebase.driveToPose(STRAIGHT_POSE),
                        drivebase.driveToPoseDeferredWithFlip(FRONT_POSE),
                        // arm.goToWristAngleCommand(WristAngle.DEPLOY),
                        // intake.in(-1),
                        // drivebase.driveToPose(STRAIGHT_POSE),
                        drivebase.driveToPoseDeferredWithFlip(MIDDLE_ZONE_POSE2D),
                        drivebase.driveToPoseDeferredWithFlip(LEFT_AUTO_RETURN_POSE),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_TRENCH_SHOOT_POSE)
                        // intake.in(0.0),
                        // shooter.spinToRPM(3000),
                        // Commands.waitUntil(() -> shooter.atSpeed(3000, 100)),
                        // indexer.out(0.8)
                        );

        Command middleDepotOutpost =
                Commands.sequence(
                        Commands.runOnce(
                                () -> drivebase.resetOdometryDeferredFlip(MIDDLE_AUTO_START_POSE)),
                        // arm.goToWristAngleCommand(WristAngle.DEPLOY),
                        // intake.in(-1),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_POSE),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_COLLECT_POSE),
                        drivebase.driveToPoseDeferredWithFlip(MIDDLE_SHOOT_POSE),
                        drivebase.driveToPoseDeferredWithFlip(OUTPOST_ZONE_POSE2D),
                        drivebase.driveToPoseDeferredWithFlip(MIDDLE_SHOOT_POSE)
                        //     drivebase.driveToPose(OUTPOST_TRENCH_SHOOT_POSE)
                        // intake.in(0.0),
                        // shooter.spinToRPM(3000),
                        // Commands.waitUntil(() -> shooter.atSpeed(3000, 100)),
                        // indexer.out(0.8)
                        );
        Command middleDepotOutpostWayPoints =
                Commands.sequence(
                        Commands.runOnce(
                                () -> drivebase.resetOdometryDeferredFlip(MIDDLE_AUTO_START_POSE)),
                        // arm.goToWristAngleCommand(WristAngle.DEPLOY),
                        // intake.in(-1),
                        drivebase.followWaypointsDeferred(DEPOT_POSE, DEPOT_PRE_POSE),
                        Commands.print("finished with waypoints"),
                        drivebase.driveToPoseDeferredWithFlip(DEPOT_COLLECT_POSE),
                        drivebase.driveToPoseDeferredWithFlip(MIDDLE_SHOOT_POSE),
                        drivebase.driveToPoseDeferredWithFlip(OUTPOST_ZONE_POSE2D),
                        drivebase.driveToPoseDeferredWithFlip(MIDDLE_SHOOT_POSE)
                        //     drivebase.driveToPose(OUTPOST_TRENCH_SHOOT_POSE)
                        // intake.in(0.0),
                        // shooter.spinToRPM(3000),
                        // Commands.waitUntil(() -> shooter.atSpeed(3000, 100)),
                        // indexer.out(0.8)
                        );
        autoChooser.addOption("drivestraight", driveStraight);
        autoChooser.addOption("outpostNeutralZone", outpostNeutralZone);
        autoChooser.addOption("depotNeutralZone", depotNeutralZone);
        autoChooser.addOption("middleDepotOutpost", middleDepotOutpost);
        autoChooser.addOption("middleDepotOutpostWayPoints", middleDepotOutpostWayPoints);
        autoChooser.setDefaultOption("donothing", Commands.none());
        SmartDashboard.putData("Autos/Selector", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
