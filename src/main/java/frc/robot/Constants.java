package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
    public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(32);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(32);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),       // Front Left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),      // Front Right
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),      // Back Left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));    // Back Right

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2 - Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI - Math.PI / 2;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = 0.0;

    // Driving Motor Prefix = 1x
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 10;
    public static final int BACK_RIGHT_DRIVING_CAN_ID = 11;
    public static final int BACK_LEFT_DRIVING_CAN_ID = 12;
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 13;

    // Turning Motor Prefix = 2x
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 20;
    public static final int BACK_RIGHT_TURNING_CAN_ID = 21;
    public static final int BACK_LEFT_TURNING_CAN_ID = 22;
    public static final int FRONT_LEFT_TURNING_CAN_ID = 23;

    public static final boolean GYRO_REVERSED = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
            / DRIVING_MOTOR_REDUCTION;

    public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION; // Meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION) / 60.0; // Meters per second

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // Radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // Radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // Radians

    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    public static final double TURNING_P = 0.25;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // Amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // Amps
  }

  public static final class AutoConstants {
    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;

    public static final double TURNING_P = 0.25;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
  }

  public static final class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DRIVE_DEADBAND = 0.02;
  }

  public static final class NeoMotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  }

  public static final class Tabs {
    public static final ShuffleboardTab MATCH = Shuffleboard.getTab("Match");
  }
}
