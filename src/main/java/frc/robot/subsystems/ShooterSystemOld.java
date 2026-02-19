package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

//indexer green wheels CIM motor 
//bottom shooter black  wheels NEO motor
//top shooter orange wheels NEO motor

public class ShooterSystemOld extends SubsystemBase {

  private final SparkMax indexerMotor = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax topShooterMotor = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax bottomShooterMotor = new SparkMax(4, MotorType.kBrushless);

 
  
  private final SmartMotorControllerConfig shooterConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0, RPM.of(5780), RPM.per(Second).of(5780))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("shooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withFollowers(Pair.of(bottomShooterMotor, true))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController shooterWrapper = new SparkWrapper(topShooterMotor,
      DCMotor.getNEO(2),
      shooterConfig);
  private final FlyWheelConfig shooterFlyWheelConfig = new FlyWheelConfig(shooterWrapper)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Mass of the flywheel. 6 4 inch stealth wheels + 1.3 pound shaft. each wheel is 0.3 pounds
      .withMass(Pounds.of(1.3 + 6 * 0.3))
      .withTelemetry("shooterMech", TelemetryVerbosity.HIGH);
  private final FlyWheel ShooterFlyWheel = new FlyWheel(shooterFlyWheelConfig);

  // this is for green wheels

   private final SmartMotorControllerConfig indexerConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0, RPM.of(5310), RPM.per(Second).of(5310))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("indexerMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController indexerWrapper = new SparkWrapper(indexerMotor,
      DCMotor.getCIM(1),
      indexerConfig);
  private final FlyWheelConfig indexerFlyWheelConfig = new FlyWheelConfig(indexerWrapper)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(3))
      // Mass of the flywheel. 6 4 inch stealth wheels + 1.3 pound shaft. each wheel is 0.17 pounds
      .withMass(Pounds.of(1.3 + 6 * 0.17))
      .withTelemetry("indexerMech", TelemetryVerbosity.HIGH);
  private final FlyWheel indexerFlyWheel = new FlyWheel(indexerFlyWheelConfig);

  



  public ShooterSystemOld() {
  }

  // /**
  //  * Gets the current velocity of the shooter.
  //  *
  //  * @return FlyWheel velocity.
  //  */
  // public AngularVelocity getVelocity() {
  //   return shooter.getSpeed();
  // }

  // /**
  //  * Set the shooter velocity.
  //  *
  //  * @param speed Speed to set.
  //  * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
  //  */
  // public Command setVelocity(AngularVelocity speed) {
  //   return shooter.setSpeed(speed);
  // }

  // /**
  //  * Set the dutycycle of the shooter.
  //  *
  //  * @param dutyCycle DutyCycle to set.
  //  * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
  //  */
  // public Command set(double dutyCycle) {
  //   return shooter.set(dutyCycle);
  // }

  // public Command setDutyCycle(Supplier<Double> dutyCycle) {
  //   return shooter.set(dutyCycle);
  // }

  // // public Command setVelocity(Supplier<AngularVelocity> speed) {return
  // // shooter.run(speed);}

  // @Override
  // public void simulationPeriodic() {
  //   shooter.simIterate();
  // }

  // @Override
  // public void periodic() {
  //   shooter.updateTelemetry();
  // }

  // // public void setRPM(LinearVelocity newHorizontalSpeed)
  // // {
  // // shooter.setMeasurementVelocitySetpoint(newHorizontalSpeed);
  // // }

  // public boolean readyToShoot(AngularVelocity tolerance) {
  //   if (topShooterWrapper.getMechanismSetpointVelocity().isEmpty()) {
  //     return false;
  //   }
  //   return topShooterWrapper.getMechanismVelocity().isNear(topShooterWrapper.getMechanismSetpointVelocity().orElseThrow(), tolerance);
  // }
}