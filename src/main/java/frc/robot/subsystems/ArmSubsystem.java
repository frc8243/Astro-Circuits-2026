package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armWristMotor = new SparkMax(6, MotorType.kBrushless);
    private final SparkClosedLoopController wristPidController =
            armWristMotor.getClosedLoopController();
    private final double kWristKS = 0;
    private final double kWristKG = 0.07;
    private final double kWristKV = 1.63;
    private final ArmFeedforward mWristFeedForward =
            new ArmFeedforward(kWristKS, kWristKG, kWristKV);
    private final SparkMaxConfig sparkMaxConfigWrist = new SparkMaxConfig();
    private final double wristEncoderPositionFactor = 1 / 83.0 * 2 * Math.PI;
    private final double wristP = 0.2 / (Math.PI / 2);
    private final double wristI = 0;
    private final double wristD = 0;
    private final RelativeEncoder wristEncoder = armWristMotor.getEncoder();

    /** Creates a new ArmWrist. */
    private static final SparkMaxConfig.IdleMode wristIdleMode = SparkBaseConfig.IdleMode.kBrake;

    public ArmSubsystem() {
        sparkMaxConfigWrist
                .inverted(false)
                .idleMode(wristIdleMode)
                .smartCurrentLimit(40)
                .softLimit
                .forwardSoftLimit(110)
                .reverseSoftLimit(-30)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);

        sparkMaxConfigWrist
                .encoder
                .positionConversionFactor(wristEncoderPositionFactor)
                .velocityConversionFactor(wristEncoderPositionFactor / 60);

        sparkMaxConfigWrist
                .closedLoop
                .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
                .pid(wristP, wristI, wristD, ClosedLoopSlot.kSlot1)
                .outputRange(-1.0, 1.0);

        armWristMotor.configure(
                sparkMaxConfigWrist,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public enum WristAngle {
        STOW(Units.degreesToRadians(-20)),

        DEPLOY(Units.degreesToRadians(100));

        private final double m_angle;

        WristAngle(double angle) {
            m_angle = angle;
        }

        public double getAngle() {
            return m_angle;
        }
    }

    private WristAngle angleEnum = WristAngle.STOW;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmWrist/Encoder Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Angle", angleEnum.getAngle());
        SmartDashboard.putString("Arm/State", "" + angleEnum);
    }

    private void goToWristAngle(double angle) {
        double ffCalc = mWristFeedForward.calculate((angle), 0.0);
        wristPidController.setSetpoint(
                angle, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot1, ffCalc);
    }

    public Command goToWristAngleCommand(WristAngle angleEnum) {
        return this.run(
                () -> {
                    goToWristAngle(angleEnum.getAngle());
                    this.angleEnum = angleEnum;
                    System.out.println("Move Wrist to " + angleEnum.toString());
                });
    }

    public void stop() {
        armWristMotor.set(0.0);
    }

    public void manualControl(double velocity) {
        armWristMotor.set(velocity * 0.5);
        SmartDashboard.putNumber("wrist/manualSpeed", velocity);
    }
}
