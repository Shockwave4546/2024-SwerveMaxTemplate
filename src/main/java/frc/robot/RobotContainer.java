package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.pose.VisionSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.commands.ChaseTagCommand;
import frc.robot.swerve.commands.ResetPoseCommand;
import frc.robot.swerve.commands.SetXCommand;

public class RobotContainer {
  protected final VisionSubsystem vision = new VisionSubsystem();
  protected final SwerveSubsystem swerve = new SwerveSubsystem();
  protected final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(swerve, vision);
  protected final CommandXboxController driverController = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  protected final AutoManager auto = new AutoManager(swerve, poseEstimator);

  public RobotContainer() {
    vision.setPoseEstimator(poseEstimator);

    configureButtonBindings();

    if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
      PPLibTelemetry.enableCompetitionMode();
    }
  }

  private void configureButtonBindings() {
    driverController.a().whileTrue(new ChaseTagCommand(vision, poseEstimator, swerve));

    driverController.b().onTrue(new ResetPoseCommand(poseEstimator));
    driverController.x().onTrue(new SetXCommand(swerve));
  }
}