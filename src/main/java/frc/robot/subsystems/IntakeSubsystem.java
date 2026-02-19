package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public static final double kWristMomentOfInertia = 0.00032; // kg * m^2

  private final SparkMax m_rollerMotor = new SparkMax(5, MotorType.kBrushless);

  private final DCMotor m_rollerMotorGearbox = DCMotor.getNEO(1);

  private final FlywheelSim m_rollerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
      m_rollerMotorGearbox,
      kWristMomentOfInertia,
      1), m_rollerMotorGearbox, 1.0 / 4096.0);

  private final SparkMaxSim m_rollerMotorSim = new SparkMaxSim(m_rollerMotor, m_rollerMotorGearbox);

  public IntakeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        .smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);
    m_rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    m_rollerSim.setInput(m_rollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_rollerSim.update(0.02);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    // m_encoderSim.setDistance(m_coralArmSim.getAngleRads());

    m_rollerMotorSim.iterate(m_rollerSim.getAngularVelocityRPM(),
        RoboRioSim.getVInVoltage(),
        // Simulated battery voltage, in Volts
        0.02);

  }

  public Command setIntakeRoller(double speed) {
    return runOnce(() -> {
      m_rollerMotor.set(speed);
    });
  }

  public Command out(double speed) {
    return setIntakeRoller(speed * -1);
  }

  public Command in(double speed) {
    return setIntakeRoller(speed);
  }

  public Command stop() {
    return setIntakeRoller(0);
  }

  public Current getCurrent() {
    return Amps.of(m_rollerMotor.getOutputCurrent());
  }

  public boolean outtaking() {
    if (getCurrentCommand() != null)
      return getDutycycle() < 0.0 || getCurrentCommand().getName().equals("Outtake");
    return getDutycycle() < 0.0;
  }

  public double getDutycycle() {
    return m_rollerMotor.getAppliedOutput();
  }
}