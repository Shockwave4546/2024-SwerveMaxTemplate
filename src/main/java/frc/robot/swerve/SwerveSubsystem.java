package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.shuffleboard.ShuffleboardBoolean;
import frc.robot.shuffleboard.ShuffleboardSpeed;

import static frc.robot.Constants.Tabs.MATCH;

public class SwerveSubsystem extends SubsystemBase {
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
          DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
          DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
          DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
          false,
          "FL"
  );

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
          DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
          DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
          false,
          "FR"
  );

  private final MAXSwerveModule backLeft = new MAXSwerveModule(
          DriveConstants.BACK_LEFT_DRIVING_CAN_ID,
          DriveConstants.BACK_LEFT_TURNING_CAN_ID,
          DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
          false,
          "BL"
  );

  private final MAXSwerveModule backRight = new MAXSwerveModule(
          DriveConstants.BACK_RIGHT_DRIVING_CAN_ID,
          DriveConstants.BACK_RIGHT_TURNING_CAN_ID,
          DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
          false,
          "BR"
  );

  private final AHRS gyro = new AHRS();
  private final ShuffleboardSpeed driveSpeedMultiplier = new ShuffleboardSpeed(MATCH, "Drive Speed Multiplier", DriveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER)
          .withSize(5, 2).withPosition(0, 4);
  private final ShuffleboardSpeed rotSpeedMultiplier = new ShuffleboardSpeed(MATCH, "Rot Speed Multiplier", DriveConstants.DEFAULT_ROT_SPEED_MULTIPLIER)
          .withSize(5, 2).withPosition(5, 4);
  private final ShuffleboardBoolean isFieldRelative = new ShuffleboardBoolean(MATCH, "Is Field Relative?", true)
          .withSize(3, 2).withPosition(6, 0);
  private boolean isX = false;

  public SwerveSubsystem() {
    MATCH.add("Gyro", gyro).withSize(3, 3).withPosition(0, 0);
    resetEncoders();
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
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean useDefaultSpeeds) {
    if (isX) {
      setX();
      return;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    final double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND * (useDefaultSpeeds ? DriveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER : driveSpeedMultiplier.get());
    final double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND * (useDefaultSpeeds ? DriveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER : driveSpeedMultiplier.get());
    final double rotDelivered = rotSpeed * DriveConstants.MAX_ANGULAR_SPEED * (useDefaultSpeeds ? DriveConstants.DEFAULT_ROT_SPEED_MULTIPLIER : rotSpeedMultiplier.get());

    final var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeadingRotation2d())
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }

  /**
   * Stops the robot.
   */
  public void stop() {
    drive(0.0, 0.0, 0.0, false, false);
  }

  /**
   * Overridden drive function for PathPlanner autonomous. It's also important to note that autonomous drives
   * given robot relative ChassisSpeeds (not field relative).
   *
   * @param speeds Speed to drive.
   */
  public void driveAutonomous(ChassisSpeeds speeds) {
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

  public SwerveModulePosition[] getEstimatedPositions() {
    return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
    };
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
   *  Zeroes the gyro of the robot.
   */
  public void zeroGyro() {
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
