package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private RelativeEncoder shooterRightEncoder;
    private SparkClosedLoopController shooterRightPIDController;
    public static final double kWristMomentOfInertia = 0.00032; // kg * m^2
    private final ClosedLoopConfig closedLoopConfigShooterRight = new ClosedLoopConfig();
    private final SparkMax m_leftRollerMotor = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax m_rightRollerMotor = new SparkMax(4, MotorType.kBrushless);
    private double targetRPM = 0;
    private final DCMotor m_rollerMotorGearbox = DCMotor.getNEO(2);

    private final FlywheelSim m_rollerSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            m_rollerMotorGearbox, kWristMomentOfInertia, 1.0 / 4.0),
                    m_rollerMotorGearbox,
                    1.0 / 4096.0);

    // private final SparkMaxSim m_rollerMotorSim = new SparkMaxSim(m_rollerMotor,
    // m_rollerMotorGearbox);
    // private final SparkMaxSim m_rollerMotorSim = new SparkMaxSim(m_rollerMotor,
    // m_rollerMotorGearbox);
    private ClosedLoopConfig shooterRightMotorConfig = new ClosedLoopConfig();

    public ShooterSubsystem() {
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.inverted(false).smartCurrentLimit(40);
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.encoder.positionConversionFactor(1.0 / (1.5));
        rightConfig.encoder.velocityConversionFactor(1.0 / (1.5));
        rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightConfig.closedLoop.pid(.1, 0, 0);
        rightConfig.closedLoop.outputRange(-1.0, 1.0);
        closedLoopConfigShooterRight.feedForward.sva(0, 0.57 * 0.508 * (60 / 6.2832), 0);

        rightConfig.apply(closedLoopConfigShooterRight);
        shooterRightPIDController = m_rightRollerMotor.getClosedLoopController();
        shooterRightEncoder = m_rightRollerMotor.getEncoder();

        m_rightRollerMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(m_rightRollerMotor, true);
        leftConfig.smartCurrentLimit(40);

        m_leftRollerMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // TODO: Set the default command, if any, for this subsystem by calling
        // setDefaultCommand(command) done
        // in the constructor or in the robot coordination class, such as
        // RobotContainer.
        // Also, you can call addChild(name, sendableChild) to associate sendables with
        // the subsystem

        // such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void periodic() {
        double currentRPM = shooterRightEncoder.getVelocity();

        SmartDashboard.putNumber("Shooter/CurrentRPM", currentRPM);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/RightCurrent", m_rightRollerMotor.getOutputCurrent());
        SmartDashboard.putNumber(
                "Shooter/RightVoltage",
                m_rightRollerMotor.getBusVoltage() * m_rightRollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/LeftCurrent", m_leftRollerMotor.getOutputCurrent());
        SmartDashboard.putNumber(
                "Shooter/LeftVoltage",
                m_leftRollerMotor.getBusVoltage() * m_leftRollerMotor.getAppliedOutput());
    }

    @Override
    public void simulationPeriodic() {}

    public boolean atSpeed(double targetRPM, double tolerance) {
        return Math.abs(shooterRightEncoder.getVelocity() - targetRPM) < tolerance;
    }

    public void setVelocity(double speed) {

        shooterRightPIDController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void defaultBehavior() {
        shooterRightPIDController.setSetpoint(0, ControlType.kVelocity);
    }

    public Command defaultBehaviorCommand() {
        return this.run(() -> defaultBehavior()).withName("shooterDefaultBehavior");
    }

    public Command runShooterCommand() {
        return this.run(() -> setVelocity(1000)).withName("setVelocity");
    }

    public Command spinToRPM(double rpm) {
        targetRPM = rpm;
        return runEnd(() -> setVelocity(rpm), () -> setVelocity(0));
    }

    public Command setShooterSpeed(double speed) {
        return startEnd(
                () -> {
                    System.out.println("Speed = " + speed);
                    m_rightRollerMotor.set(speed);
                },
                () -> {
                    m_rightRollerMotor.set(0);
                });
    }

    public Command out(double speed) {
        return setShooterSpeed(speed * -1);
    }

    public Command in(double speed) {
        return setShooterSpeed(speed);
    }

    public Command stop() {
        return setShooterSpeed(0);
    }

    public Current getCurrent() {
        return Amps.of(m_leftRollerMotor.getOutputCurrent());
    }

    public boolean outtaking() {
        if (getCurrentCommand() != null)
            return getDutycycle() < 0.0 || getCurrentCommand().getName().equals("Outtake");
        return getDutycycle() < 0.0;
    }

    public double getDutycycle() {
        return m_leftRollerMotor.getAppliedOutput();
    }
}
