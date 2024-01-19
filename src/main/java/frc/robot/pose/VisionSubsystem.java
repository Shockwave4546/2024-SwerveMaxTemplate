package frc.robot.pose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("OV9281");
  private PoseEstimatorSubsystem poseEstimator;

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
      throw new RuntimeException("You can only set the poseEstimator once.");
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
   * @return true if the camera has found targets, which have an ambiguity of less than 0.2.
   */
  public boolean hasViableTarget() {
    final var pipeline = camera.getLatestResult();
    return pipeline.hasTargets() && pipeline.getBestTarget().getPoseAmbiguity() < 0.2;
  }

  /**
   * Note: please call #hasViableTarget() before calling this method.
   *
   * @return the best target found by the camera.
   *         throws RuntimeException if no targets are found.
   */
  public PhotonTrackedTarget getTag() {
    final var pipeline = camera.getLatestResult();
    if (!pipeline.hasTargets()) throw new RuntimeException("Call this method only when targets are found.");
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
    return poseEstimator.getPose3d().transformBy(Constants.VisionConstants.ROBOT_TO_CAMERA);
  }

  /**
   * Note: please call #hasViableTarget() before calling this method.
   * Would later use by doing CameraPose.transformBy(#getCameraToTagTransform())
   *
   * @return the [Transform3d] to transform the Camera pose to the Tag pose.
   *         throws RuntimeException if no targets are found.
   */
  public Transform3d getCameraToTagTransform() {
    final var pipeline = camera.getLatestResult();
    if (!pipeline.hasTargets()) throw new RuntimeException("Call this method only when targets are found.");
    final var tag = pipeline.getBestTarget();
    return tag.getBestCameraToTarget().plus(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, Math.PI)));
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