package frc.robot.pose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(Vision.FRONT_CAMERA_NAME);
  private PoseEstimatorSubsystem poseEstimator;

  @SuppressWarnings("resource")
  public VisionSubsystem() {
    final var tab = Shuffleboard.getTab("Odometry");
    tab.addBoolean("Has Viable Target", this::hasViableTarget).withSize(3, 2).withPosition(0, 0);
  }

  /**
   * Pretend like you don't see this.
   * @param poseEstimator the poseEstimator to set.
   */
  public void setPoseEstimator(PoseEstimatorSubsystem poseEstimator) {
    if (this.poseEstimator != null) {
      DriverStation.reportError("The PoseEstimator can only be initialized once!", false);
      return;
    }

    this.poseEstimator = poseEstimator;
  }

  /**
   * @return the latest timestamp of the pipeline.
   */
  public double getLatestPipelineTimestamp() {
    return camera.getLatestResult().getTimestampSeconds();
  }

  /**
   * Note: please call #hasViableTarget() before calling this method.
   *
   * @return the best target found by the camera.
   *         null if no targets are found.
   */
  public PhotonTrackedTarget getTag() {
    final var pipeline = camera.getLatestResult();
    if (!hasViableTarget()) {
      DriverStation.reportError("No targets found!", false);
      return null;
    }

    return pipeline.getBestTarget();
  }

  /**
   * Note: please call #hasViableTarget() before calling this method.
   * @return the physical location of the tag, relative to the center of the robot.
   */
  public Pose3d getTagRelativeToCenterPose() {
    final var cameraPose = getCameraRelativeToCenterPose();
    final var cameraToTagTransform = getCameraToTagTransform();
    return cameraPose.transformBy(cameraToTagTransform);
  }

  /**
   * Ignores height (z-axis).
   *
   * @return the physical location of the camera on the robot, relative to the center of the robot.
   */
  public Pose3d getCameraRelativeToCenterPose() {
    if (poseEstimator == null) throw new NullPointerException("Initialize the poseEstimator before calling this method.");
    return poseEstimator.getPose3d().transformBy(Vision.ROBOT_TO_CAMERA);
  }

  /**
   * Note: please call #hasViableTarget() before calling this method.
   * Would later use by doing CameraPose.transformBy(#getCameraToTagTransform())
   *
   * @return the [Transform3d] to transform the Camera pose to the Tag pose.
   *         null if no targets are found.
   */
  public Transform3d getCameraToTagTransform() {
    return getTag().getBestCameraToTarget();
  }

  /**
   * @return true if the camera has found targets, which have an ambiguity of less than 0.2.
   */
  public boolean hasViableTarget() {
    final var pipeline = camera.getLatestResult();
    return pipeline.hasTargets() && pipeline.getBestTarget().getPoseAmbiguity() < Vision.MAXIMUM_AMBIGUITY;
  }

  /**
   * Disclaimer: Don't use this method raw, use an encapsulated method.
   *
   * @return the PhotonCamera.
   */
  public PhotonCamera getPhotonCamera() {
    return camera;
  }
}