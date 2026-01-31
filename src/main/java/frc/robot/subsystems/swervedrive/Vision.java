package frc.robot.subsystems.swervedrive;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;


/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision
{

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout                     = AprilTagFieldLayout.loadField(
      AprilTagFields.k2026RebuiltAndymark);
  /**
   * Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
   */
  private final       double              maximumAmbiguity                = 0.25;
  /**
   * Photon Vision Simulation
   */
  public              VisionSystemSim     visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the april tag.
   */
  private             double              longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private             Supplier<Pose2d>    currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private             Field2d             field2d;


  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */


   private final String limelightName;

   // how much to trust LL pose (tune later)
   private final Matrix<N3,N1>visionStdDevs =
VecBuilder.fill(0.7, 0.7, Math.toRadians(15));

   public Vision(String limelightName) {
    this.limelightName = limelightName;
   }

   /** Read AprilTag pose and send it to YAGSL*/
   public void updatePose(SwerveDrive drive){

    //MegaTag2 is best
    var est = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    if (est == null) return;
    if (est.tagCount < 1) return;

    Pose2d pose = est.pose;

    double timestamp =
    Timer.getFPGATimestamp() -
    (est.latency_capture + est.latency_pipeline) / 1000.0;

    drive.addVisionMeasurement(pose, timestamp,visionStdDevs);
}
}