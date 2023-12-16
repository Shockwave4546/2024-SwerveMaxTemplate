package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.ModulePosition;
import frc.shuffleboard.ShuffleboardSpeed;

import static frc.robot.Constants.Tabs.MATCH;

public class DriveSubsystem extends SubsystemBase {
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
  private final ShuffleboardSpeed DRIVE_SPEED_MULTIPLIER = new ShuffleboardSpeed(MATCH, "Drive Speed Multiplier", 0.8);
  private final ShuffleboardSpeed ROT_SPEED_MULTIPLIER = new ShuffleboardSpeed(MATCH, "Rot Speed Multiplier", 1.0);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
          DriveConstants.DRIVE_KINEMATICS,
          getHeadingRotation2d(),
          new SwerveModulePosition[] {
                  frontLeft.getPosition(),
                  frontRight.getPosition(),
                  backLeft.getPosition(),
                  backRight.getPosition()
          });

  public DriveSubsystem() {
    MATCH.add("Gyro", gyro);
    zeroHeading();

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
    // Update the odometry in the periodic block
    odometry.update(
            getHeadingRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });
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

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
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
    // Convert the commanded speeds into the correct units for the drivetrain
    final double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND * DRIVE_SPEED_MULTIPLIER.get();
    final double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND * DRIVE_SPEED_MULTIPLIER.get();
    final double rotDelivered = rotSpeed * DriveConstants.MAX_ANGULAR_SPEED * ROT_SPEED_MULTIPLIER.get();

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
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    setModuleStates(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    );
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
   * Sets the swerve ModuleStates.
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
   * @return The robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getRawAngleDegrees()).getDegrees();
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
    return (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0) * gyro.getFusedHeading();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0) * gyro.getRate();
  }
}
