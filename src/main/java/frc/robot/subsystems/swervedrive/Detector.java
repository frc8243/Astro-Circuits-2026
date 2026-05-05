package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.LimelightHelpers.RawDetection;

/**
 * Game-piece (object detection) wrapper around a single Limelight (intended for LL3 running a
 * neural detector pipeline).
 *
 * <p>This class does NOT contribute to pose estimation. Use it for tx/ty driven alignment, target
 * picking, and pipeline switching. The {@code limelightName} should match the suffix configured on
 * the Limelight web UI; e.g. a Limelight whose hostname is {@code limelight-detect} should be
 * constructed with name {@code "detect"}.
 */
public class Detector {

    private final String limelightName;
    private final String limelightHostname;

    public Detector(String limelightName) {
        this.limelightName = limelightName;
        this.limelightHostname =
                "limelight" + (!limelightName.isEmpty() ? "-" + limelightName : "");
    }

    public String getName() {
        return limelightName;
    }

    /** True when the active pipeline reports a valid target. */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /** Horizontal offset from crosshair to target, degrees. + = right of center. */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    /** Vertical offset from crosshair to target, degrees. + = above center. */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /** Target area as a percentage of the image (0..100). Useful for rough distance gating. */
    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    /** Class name reported by the active detector pipeline (e.g. game piece label). */
    public String getDetectedClass() {
        return LimelightHelpers.getDetectorClass(limelightName);
    }

    public int getDetectedClassIndex() {
        return LimelightHelpers.getDetectorClassIndex(limelightName);
    }

    /** Raw per-detection results (multiple targets, with corners). Empty array if none. */
    public RawDetection[] getRawDetections() {
        return LimelightHelpers.getRawDetections(limelightName);
    }

    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    /** Publish current detection state to SmartDashboard. Call from a periodic method. */
    public void log() {
        boolean tv = hasTarget();
        SmartDashboard.putBoolean("Detector/" + limelightHostname + "/HasTarget", tv);
        if (tv) {
            SmartDashboard.putNumber("Detector/" + limelightHostname + "/TX", getTX());
            SmartDashboard.putNumber("Detector/" + limelightHostname + "/TY", getTY());
            SmartDashboard.putNumber("Detector/" + limelightHostname + "/TA", getTA());
            SmartDashboard.putString("Detector/" + limelightHostname + "/Class", getDetectedClass());
        }
    }
}
