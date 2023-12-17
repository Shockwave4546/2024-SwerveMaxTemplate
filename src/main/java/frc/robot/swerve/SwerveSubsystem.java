package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.shuffleboard.ShuffleboardBoolean;
import frc.robot.shuffleboard.ShuffleboardSpeed;
import org.photonvision.PhotonCamera;

import java.io.IOException;

import static frc.robot.Constants.Tabs.MATCH;

public class SwerveSubsystem extends SubsystemBase {
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
          DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
          DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
          DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
          false,
          Shuffleboard.getTab("Front Left Motors")
  );

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
          DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
          DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
          false,
          Shuffleboard.getTab("Front Right Motors")
  );

  private final MAXSwerveModule backLeft = new MAXSwerveModule(
          DriveConstants.BACK_LEFT_DRIVING_CAN_ID,
          DriveConstants.BACK_LEFT_TURNING_CAN_ID,
          DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
          false,
          Shuffleboard.getTab("Back Left Motors")
  );

  private final MAXSwerveModule backRight = new MAXSwerveModule(
          DriveConstants.BACK_RIGHT_DRIVING_CAN_ID,
          DriveConstants.BACK_RIGHT_TURNING_CAN_ID,
          DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
          false,
          Shuffleboard.getTab("Back Right Motors")
  );

  private final AHRS gyro = new AHRS();
  private final ShuffleboardSpeed driveSpeedMultiplier = new ShuffleboardSpeed(MATCH, "Drive Speed Multiplier", 0.8);
  private final ShuffleboardSpeed rotSpeedMultiplier = new ShuffleboardSpeed(MATCH, "Rot Speed Multiplier", 1.0);
  private final ShuffleboardBoolean isFieldRelative = new ShuffleboardBoolean(MATCH, "Is Field Relative?", true);

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
          getHeadingRotation2d(),
          new SwerveModulePosition[] {
                  frontLeft.getPosition(),
                  frontRight.getPosition(),
                  backLeft.getPosition(),
                  backRight.getPosition()
          },
          new Pose2d(),
          STATE_STD_DEVS,
          VISION_MEASUREMENT_STD_DEVS
  );

  private final PhotonCamera camera;
  private final AprilTagFieldLayout layout;
  private double previousPipelineTimestamp = 0.0;
  private boolean isX = false;

  public SwerveSubsystem(PhotonCamera camera) {
    this.camera = camera;
    try {
      this.layout = AprilTagFieldLayout.loadFromResource("/deploy/exampleAprilLayout.json");
      layout.setOrigin(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    MATCH.add("Gyro", gyro);
    // The "forward" direction will always be relative to the starting position of the Robot.
    zeroHeading();
    resetEncoders();

    // Ensure this is called after all initialization is complete.
    AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRelativeChassisSpeed,
            this::drive,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(AutoConstants.DRIVING_P, AutoConstants.DRIVING_I, AutoConstants.DRIVING_D),
                    new PIDConstants(AutoConstants.TURNING_P, AutoConstants.TURNING_I, AutoConstants.TURNING_D),
                    DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                    DriveConstants.WHEEL_BASE / 2,
                    new ReplanningConfig()
            ),
            this
    );
  }

  @Override public void periodic() {
    poseEstimator.update(
            getHeadingRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    final var pipelineResult = camera.getLatestResult();
    final var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      final var target = pipelineResult.getBestTarget();
      final var fiducialId = target.getFiducialId();
      final var targetPose = layout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= 0.2 && fiducialId >= 0 && targetPose.isPresent()) {
        final var camToTarget = target.getBestCameraToTarget();
        final var camPose = targetPose.get().transformBy(camToTarget.inverse());
        final var visioMeasurement = camPose.transformBy(DriveConstants.CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visioMeasurement.toPose2d(), resultTimestamp);
      }
    }
  }

  /**
   * It's important the SwerveModules are passed in with respect to the Kinematics construction.
   *
   * @return chassis speed relative to the robot.
   */
  public ChassisSpeeds getRelativeChassisSpeed() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
    );
  }

  private void setX() {
    setModuleStates(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    );
  }

  public void toggleX() {
    this.isX = !this.isX;
  }

  public boolean isFieldRelative() {
    return isFieldRelative.get();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
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
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    if (isX) {
      setX();
      return;
    }
    // Convert the commanded speeds into the correct units for the drivetrain
    final double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND * driveSpeedMultiplier.get();
    final double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND * driveSpeedMultiplier.get();
    final double rotDelivered = rotSpeed * DriveConstants.MAX_ANGULAR_SPEED * rotSpeedMultiplier.get();

    final var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeadingRotation2d())
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }

  /**
   * Overridden drive function for PathPlanner autonomous. It's also important to note that autonomous drives
   * given robot relative ChassisSpeeds (not field relative).
   *
   * @param speeds Speed to drive.
   */
  private void drive(ChassisSpeeds speeds) {
    // For some reason, PathPlanner is giving me opposite directions, so use this temporary fix.
    final var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds.times(-1.0));
    setModuleStates(swerveModuleStates);
  }

  /**
   * @param position The relative position of the SwerveModule.
   * @return the MAXSwerveModule associated with the ModulePosition.
   */
  public MAXSwerveModule getModule(ModulePosition position) {
    switch (position) {
      case FRONT_LEFT -> { return frontLeft; }
      case FRONT_RIGHT -> { return frontRight; }
      case BACK_LEFT -> { return backLeft; }
      case BACK_RIGHT ->  { return backRight; }
      default -> throw new RuntimeException("I don't even know what you put in here...");
    }
  }

  /**
   * Sets the swerve ModuleStates. (FL, FR, BL, BR)
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState... desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /**
   *  Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading as a Rotation2d.
   */
  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getRawAngleDegrees());
  }

  /**
   * The default SwerveMax template has an issue with inverting the Gyro, so the workaround is
   * manually negating the AHRS#getAngle. This function shouldn't get called by the user.
   *
   * @return The properly negated angle in degrees.
   */
  private double getRawAngleDegrees() {
    return (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0) * gyro.getAngle();
  }
}
