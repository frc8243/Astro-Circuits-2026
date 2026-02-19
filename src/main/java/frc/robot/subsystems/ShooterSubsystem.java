package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private RelativeEncoder shooterRightEncoder;
  private SparkClosedLoopController shooterRightPIDController;
  public static final double kWristMomentOfInertia = 0.00032; // kg * m^2
  private final ClosedLoopConfig closedLoopConfigShooterRight = new ClosedLoopConfig();
  private final SparkMax m_leftRollerMotor = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax m_rightRollerMotor = new SparkMax(4, MotorType.kBrushless);

  private final DCMotor m_rollerMotorGearbox = DCMotor.getNEO(2);

  private final FlywheelSim m_rollerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
      m_rollerMotorGearbox,
      kWristMomentOfInertia,
      1.0 / 4.0), m_rollerMotorGearbox, 1.0 / 4096.0);

  // private final SparkMaxSim m_rollerMotorSim = new SparkMaxSim(m_rollerMotor,
  // m_rollerMotorGearbox);
  // private final SparkMaxSim m_rollerMotorSim = new SparkMaxSim(m_rollerMotor,
  // m_rollerMotorGearbox);

  public ShooterSubsystem() {
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .inverted(false)
        .smartCurrentLimit(40);
    // rightConfig.idleMode(IdleMode.kCoast);
    // rightConfig.encoder.positionConversionFactor(1.0/(3.0*4.0));
    // rightConfig.encoder.velocityConversionFactor(1.0/(3.0*4.0)/60.0);
    // rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // rightConfig.closedLoop.pid(.1,0,0);
    // rightConfig.closedLoop.outputRange(-1.0,1.0);
    // closedLoopConfigShooterRight.feedForward.sva(0,12/5760.0,0);
    // shooterRightMotorConfig.apply(closedLoopConfigShooterRight);
    // shooterRightPIDController = m_rightRollerMotor.getClosedLoopController();
    // shooterRightEncoder = m_rightRollerMotor.getEncoder();

    m_rightRollerMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.follow(m_rightRollerMotor, true);

    m_leftRollerMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command) done
    // in the constructor or in the robot coordination class, such as
    // RobotContainer.
    // Also, you can call addChild(name, sendableChild) to associate sendables with
    // the subsystem

    // such as SpeedControllers, Encoders, DigitalInputs, etc.
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    // m_rollerSim.setInput(m_rollerMotorSim.getAppliedOutput() *
    // RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    // m_rollerSim.update(0.02);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    // m_encoderSim.setDistance(m_coralArmSim.getAngleRads());

    // m_rollerMotorSim.iterate(m_rollerSim.getAngularVelocityRPM(),
    // RoboRioSim.getVInVoltage(),
    // Simulated battery voltage, in Volts
    // 0.02);

  }

  public void runShooter() {
    shooterRightPIDController.setSetpoint(1000, ControlType.kVelocity);
  }

  public void defaultBehavior() {
    shooterRightPIDController.setSetpoint(0, ControlType.kVelocity);
  }

  public Command defaultBehaviorCommand() {
    return this.run(() -> defaultBehavior()).withName("shooterDefaultBehavior");
  }

  public Command runShooterCommand() {
    return this.run(() -> runShooter()).withName("setVelocity");
  }

  public Command setShooterSpeed(double speed) {
    return runOnce(() -> {
      m_leftRollerMotor.set(speed);
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