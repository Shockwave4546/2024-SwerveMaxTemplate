package frc.robot.swerve;

import com.revrobotics.*;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ModuleConstants;
import frc.robot.shuffleboard.TunableSparkMaxPIDController;

import static com.revrobotics.CANSparkLowLevel.*;

public class MAXSwerveModule {
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;

  private final double chassisAngularOffset;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and SparkMaxPIDController. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a ThroughBore Encoder.
   */
  @SuppressWarnings("resource")
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean invertDrivingDirection, ShuffleboardTab tab) {
    final var drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    final var turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    drivingSparkMax.setInverted(invertDrivingDirection);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivingPIDController = drivingSparkMax.getPIDController();
    turningPIDController = turningSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
    drivingEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
    turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivingPIDController.setP(ModuleConstants.DRIVING_P);
    drivingPIDController.setI(ModuleConstants.DRIVING_I);
    drivingPIDController.setD(ModuleConstants.DRIVING_D);
    drivingPIDController.setFF(ModuleConstants.DRIVING_FF);
    drivingPIDController.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT, ModuleConstants.DRIVING_MAX_OUTPUT);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPIDController.setP(ModuleConstants.TURNING_P);
    turningPIDController.setI(ModuleConstants.TURNING_I);
    turningPIDController.setD(ModuleConstants.TURNING_D);
    turningPIDController.setFF(ModuleConstants.TURNING_FF);
    turningPIDController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT, ModuleConstants.TURNING_MAX_OUTPUT);

    drivingSparkMax.setIdleMode(ModuleConstants.DRIVING_MOTOR_IDLE_MODE);
    turningSparkMax.setIdleMode(ModuleConstants.TURNING_MOTOR_IDLE_MODE);
    drivingSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);

    // For debugging purposes.
    tab.add("(Driving) ID", drivingCANId).withSize(4, 1).withPosition(0, 0);
    tab.addNumber("(Driving Applied Duty Cycle", drivingSparkMax::getAppliedOutput).withSize(4, 1).withPosition(0, 1);
    tab.addNumber("(Driving) Applied Amperage", drivingSparkMax::getOutputCurrent).withSize(4, 1).withPosition(0, 2);
    tab.addNumber("(Driving) Temperature (C)", drivingSparkMax::getMotorTemperature).withSize(4, 1).withPosition(0, 3);
    tab.add("(Driving) PID Controller", new TunableSparkMaxPIDController(drivingPIDController)).withSize(2, 3).withPosition(4, 0);

    tab.add("(Turning) ID", turningCANId).withSize(4, 1).withPosition(0, 4);
    tab.addNumber("(Turning) Applied Duty Cycle", turningSparkMax::getAppliedOutput).withSize(4, 1).withPosition(0, 5);
    tab.addNumber("(Turning) Applied Amperage", turningSparkMax::getOutputCurrent).withSize(4, 1).withPosition(0, 6);
    tab.addNumber("(Turning) Temperature (C)", turningSparkMax::getMotorTemperature).withSize(4, 1).withPosition(0, 7);
    tab.add("(Turning) Turning PID Controller", new TunableSparkMaxPIDController(turningPIDController)).withSize(2, 3).withPosition(4, 4);
  }

  /**
   * Returns the current state of the module. A chassis angular offset is applied to the encoder position
   * to get the position relative to the chassis.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(drivingEncoder.getVelocity(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module. A chassis angular offset is applied to the encoder position
   * to get the position relative to the chassis.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    final var correctedDesiredState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))
    );

    // Optimize the reference state to avoid spinning further than 90 degrees.
    final var optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /**
   * Zeroes the SwerveModule encoder.
   */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }
}
