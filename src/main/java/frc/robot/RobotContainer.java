package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.shuffleboard.GlobalTab;
import frc.shuffleboard.ShuffleboardSpeed;

public class RobotContainer {
  protected final DriveSubsystem drive = new DriveSubsystem();
  protected final CommandXboxController driverController = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  public static final ShuffleboardSpeed SPEED_MULT = new ShuffleboardSpeed(GlobalTab.MATCH, "Speed MUlt", 0.8);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    driverController.a().onTrue(new RunCommand(drive::zeroHeading, drive));
  }

//  /**
//   * Use this to pass the autonomous command to the main {@link Robot} class.
//   *
//   * @return the command to run in autonomous
//   */
//  public Command getAutonomousCommand() {
//    // Create config for trajectory
//    TrajectoryConfig config = new TrajectoryConfig(
//            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
//            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
//            // Add kinematics to ensure max speed is actually obeyed
//            .setKinematics(DriveConstants.DRIVE_KINEMATICS);
//
//    // An example trajectory to follow. All units in meters.
//    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//            // Start at the origin facing the +X direction
//            new Pose2d(0, 0, new Rotation2d(0)),
//            // Pass through these two interior waypoints, making an 's' curve path
//            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//            // End 3 meters straight ahead of where we started, facing forward
//            new Pose2d(3, 0, new Rotation2d(0)),
//            config);
//
//    var thetaController = new ProfiledPIDController(
//            AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
//    thetaController.enableContinuousInput(-Math.PI, Math.PI);
//
//    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//            exampleTrajectory,
//            robotDrive::getPose, // Functional interface to feed supplier
//            DriveConstants.DRIVE_KINEMATICS,
//
//            // Position controllers
//            new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
//            new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
//            thetaController,
//            robotDrive::setModuleStates,
//            robotDrive);
//
//    // Reset odometry to the starting pose of the trajectory.
//    robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
//
//    // Run path following command, then stop at the end.
//    return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
//  }
}
