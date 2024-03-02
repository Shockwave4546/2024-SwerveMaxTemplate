package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.pose.ResetPoseCommand;
import frc.robot.pose.VisionSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.commands.ChaseTagCommand;
import frc.robot.swerve.commands.SetSpeedMaxCommand;
import frc.robot.swerve.commands.SetXCommand;

import static frc.robot.Constants.IO;

public class RobotContainer {
  protected final VisionSubsystem vision = new VisionSubsystem();
  protected final SwerveSubsystem swerve = new SwerveSubsystem();
  protected final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(swerve, vision);
  protected final CommandXboxController driverController = new CommandXboxController(IO.DRIVER_CONTROLLER_PORT);
  protected final AutoManager auto = new AutoManager(swerve, poseEstimator);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    vision.setPoseEstimator(poseEstimator);

    configureButtonBindings();

    if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
      PPLibTelemetry.enableCompetitionMode();
    }
  }

  private void configureButtonBindings() {
    driverController.a().whileTrue(new ChaseTagCommand(vision, poseEstimator, swerve));

    driverController.b().onTrue(new ResetPoseCommand(swerve, poseEstimator));
    driverController.x().onTrue(new SetXCommand(swerve));
    driverController.leftBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.2, 0.4));
    driverController.rightBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.4, 0.6));
  }
}