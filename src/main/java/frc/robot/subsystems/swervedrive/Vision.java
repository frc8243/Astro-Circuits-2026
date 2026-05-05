package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;

/**
 * AprilTag pose-estimation wrapper around a single Limelight (intended for LL4).
 *
 * <p>Pass the same name configured on the Limelight web UI (Settings -> Hostname). For example, a
 * Limelight whose hostname is {@code limelight-tag} should be constructed with name {@code "tag"}.
 */
public class Vision {

    /** Camera mounting offsets, all measured from robot center. */
    public static class CameraOffset {
        public final double forwardMeters;
        public final double leftMeters;
        public final double upMeters;
        public final double rollDeg;
        public final double pitchDeg;
        public final double yawDeg;

        public CameraOffset(
                double forwardMeters,
                double leftMeters,
                double upMeters,
                double rollDeg,
                double pitchDeg,
                double yawDeg) {
            this.forwardMeters = forwardMeters;
            this.leftMeters = leftMeters;
            this.upMeters = upMeters;
            this.rollDeg = rollDeg;
            this.pitchDeg = pitchDeg;
            this.yawDeg = yawDeg;
        }
    }

    private final String limelightName;
    private final String limelightHostname;
    private final StructPublisher<Pose2d> posePublisher;

    public Vision(String limelightName, CameraOffset offset) {
        this.limelightName = limelightName;
        this.limelightHostname =
                "limelight" + (!limelightName.isEmpty() ? "-" + limelightName : "");

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                offset.forwardMeters,
                offset.leftMeters,
                offset.upMeters,
                offset.rollDeg,
                offset.pitchDeg,
                offset.yawDeg);

        posePublisher =
                NetworkTableInstance.getDefault()
                        .getStructTopic("VisionPoseEstimator/" + limelightHostname, Pose2d.struct)
                        .publish();
        posePublisher.setDefault(new Pose2d());
    }

    public void updatePose(SwerveDrive drive) {
        // required for MegaTag2 to work
        LimelightHelpers.SetRobotOrientation(
                limelightName, drive.getYaw().getDegrees(), 0, 0, 0, 0, 0);

        // reject if spinning too fast (> 2 rot/sec)
        if (Math.abs(drive.getRobotVelocity().omegaRadiansPerSecond) > (2 * Math.PI * 2)) return;

        double linearSpeed =
                Math.hypot(
                        drive.getRobotVelocity().vxMetersPerSecond,
                        drive.getRobotVelocity().vyMetersPerSecond);

        // reject if driving to0 fast, > 80% of robot speed!
        if (linearSpeed > 0.8 * drive.getMaximumChassisVelocity()) return;

        var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (est == null) return;
        if (est.tagCount < 1) return;
        if (est.pose.getX() == 0 && est.pose.getY() == 0) return;

        // publish pose to NT for AdvantageScope/Shuffleboard
        posePublisher.set(est.pose);

        StringBuilder tagIDs = new StringBuilder();
        if (est.rawFiducials != null) {
            for (int i = 0; i < est.rawFiducials.length; i++) {
                if (i > 0) tagIDs.append(", ");
                tagIDs.append(est.rawFiducials[i].id);
            }
        }
        SmartDashboard.putString("Vision/" + limelightHostname + "/TagIDs", tagIDs.toString());
        // use Limelight's own stddevs instead of hardcoded values
        // layout: [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1yaw, MT2x, MT2y, MT2z, MT2roll,
        // MT2pitch, MT2yaw]
        var stddevs = LimelightHelpers.getLimelightNTDoubleArray(limelightHostname, "stddevs");
        if (stddevs == null || stddevs.length < 8) return;
        double timestamp =
                Timer.getFPGATimestamp() - (est.latency_capture + est.latency_pipeline) / 1000.0;

        drive.addVisionMeasurement(
                est.pose,
                timestamp,
                VecBuilder.fill(stddevs[6], stddevs[7], Double.POSITIVE_INFINITY));
    }
}
