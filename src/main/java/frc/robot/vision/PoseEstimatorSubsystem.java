package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.swerve.SwerveSubsystem;

import java.io.IOException;

import static frc.robot.Constants.DriveConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Pose Estimator");
    /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   * Source: <a href="https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java">...</a>
   */
  private static final Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   * Source: <a href="https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java">...</a>
   */
  private static final Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
          DriveConstants.DRIVE_KINEMATICS,
          new Rotation2d(),
          new SwerveModulePosition[] {
                  new SwerveModulePosition(0.0, new Rotation2d()),
                  new SwerveModulePosition(0.0, new Rotation2d()),
                  new SwerveModulePosition(0.0, new Rotation2d()),
                  new SwerveModulePosition(0.0, new Rotation2d())
          },
          new Pose2d(),
          STATE_STD_DEVS,
          VISION_MEASUREMENT_STD_DEVS
  );

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
//  private final AprilTagFieldLayout layout;
  private double previousPipelineTimestamp = 0.0;

  public PoseEstimatorSubsystem(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;

//    try {
//      this.layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
//      // Uses blue side as default in the event that the alliance color is null.
//      final var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : DriverStation.Alliance.Blue;
//      layout.setOrigin(alliance == DriverStation.Alliance.Blue ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
//    } catch (IOException e) {
//      throw new RuntimeException(e);
//    }

    tab.addNumber("Pose X", () -> getPose2d().getX());
    tab.addNumber("Pose Y", () -> getPose2d().getY());
    tab.addNumber("Pose Degrees", () -> getPose2d().getRotation().getDegrees());
  }


  @Override public void periodic() {
    poseEstimator.update(swerve.getHeadingRotation2d(), swerve.getEstimatedPositions());

    final var resultTimestamp = vision.getLatestPipelineTimestamp();
    if (resultTimestamp != previousPipelineTimestamp && vision.hasViableTarget()) {
      previousPipelineTimestamp = resultTimestamp;
      final var tag = vision.getTag();
      final var id = tag.getFiducialId();
      // TODO: 1/16/2024 Need to test this.
//      final var targetPose = layout.getTagPose(id);
//      if (id >= 0 && targetPose.isPresent()) {
//        final var camToTag = vision.getCameraToTagTransform();
//        final var camPose = targetPose.get().transformBy(camToTag.inverse());
//        final var visionMeasurement = camPose.transformBy(VisionConstants.CAMERA_TO_ROBOT);
//        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
//      }
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
            new Rotation2d(),
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            },
            pose);
  }

  /**
   * Zeros the heading. This sets the direction for field-centric driving.
   */
  public void zeroHeading() {
    swerve.zeroGyro();
    resetOdometry(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
  }

  /**
   * @return the Pose2d of the robot.
   */
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @return the Pose3d of the robot, with the z-axis being the heading of the robot.
   */
  public Pose3d getPose3d() {
    return new Pose3d(
            getPose2d().getX(),
            getPose2d().getY(),
            0.0,
            new Rotation3d(0.0, 0.0, getPose2d().getRotation().getRadians())
    );
  }
}