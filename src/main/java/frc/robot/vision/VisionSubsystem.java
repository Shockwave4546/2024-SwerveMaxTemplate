package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem implements Subsystem {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  private final PhotonCamera camera = new PhotonCamera("OV9281");
  private PoseEstimatorSubsystem poseEstimator;

  @Override public void periodic() {
    tab.addNumber("Latest Pipeline Timestamp", this::getLatestPipelineTimestamp);
    tab.addBoolean("Has Viable Target", this::hasViableTarget);

    if (!hasViableTarget()) return;
    tab.addNumber("Tag Pose Ambiguity", () -> getTag().getPoseAmbiguity());
    tab.addNumber("Tag Pose X", () -> getTagRelativeToCenterPose().getX());
    tab.addNumber("Tag Pose Y", () -> getTagRelativeToCenterPose().getY());
    tab.addNumber("Tag Pose Degrees", () -> getTagRelativeToCenterPose().getRotation().getZ());
  }

  /**
   * Pretend like you don't see this.
   * @param poseEstimator the poseEstimator to set.
   */
  public void setPoseEstimator(PoseEstimatorSubsystem poseEstimator) {
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
   * Call #hasViableTarget() before calling this method.
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
   * Would later use by doing CameraPose.transformBy(#getCameraToTagTransform())
   *
   * @return the [Transform3d] to transform the Camera pose to the Tag pose.
   *         throws RuntimeException if no targets are found.
   */
  public Transform3d getCameraToTagTransform() {
    final var pipeline = camera.getLatestResult();
    if (!pipeline.hasTargets()) throw new RuntimeException("Call this method only when targets are found.");
    final var tag = pipeline.getBestTarget();
    return normalizeAngle(tag.getBestCameraToTarget());
  }

  /**
   * NavX reports a [0, 2pi] angle, whereas PhotonVision reports [-pi, pi]
   * If the angle is negative add 2pi to it.
   *
   * @return a transformed angle from [-pi, pi] to [0, 2pi].
   */
  private Transform3d normalizeAngle(Transform3d transform) {
    final var currentAngle = transform.getRotation().getZ();
    return transform.plus(new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, currentAngle < 0 ? (2 * Math.PI) : 0.0)));
  }
}