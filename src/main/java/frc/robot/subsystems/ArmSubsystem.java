package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {

  private final SparkMax armMotor = new SparkMax(6, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      .withSoftLimit(Degrees.of(0), Degrees.of(120))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0/18.0))) //original numbers were 5,5,2
      .withIdleMode(MotorMode.COAST) //BrAKE?
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.1))
            .withOpenLoopRampRate(Seconds.of(0.1))

      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
    
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController motor = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
   
  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(0.5)) //22in tall
      .withMaxRobotLength(Meters.of(0.68)) //27 x 27 in chassis
      .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0))); // need to clean this up! 
  
  private ArmConfig m_config = new ArmConfig(motor)
      .withLength(Meters.of(0.307))
      .withSoftLimits(Degrees.of(0), Degree.of(120))
      .withHardLimit(Degrees.of(0), Degrees.of(120))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(2)) 
      .withMechanismPositionConfig(robotToMechanism)
      .withStartingPosition(Degrees.of(0));

  private final Arm arm = new Arm(m_config);

  public ArmSubsystem() {
  }

  public void periodic() {
    arm.updateTelemetry();
  }

  public void simulationPeriodic() {
    arm.simIterate();
  }

  public Command armCmd(double dutycycle) {
    return arm.set(dutycycle);
  }

  public Command sysId() {
    return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }
}
