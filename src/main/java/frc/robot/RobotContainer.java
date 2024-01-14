package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.commands.ApproachTargetCommand;
import frc.robot.swerve.commands.ResetGyroCommand;
import frc.robot.swerve.commands.SetXCommand;
import org.photonvision.PhotonCamera;

import java.io.IOException;

public class RobotContainer {
  protected final PhotonCamera camera = new PhotonCamera("OV9281");
  private final AprilTagFieldLayout layout;
  protected final SwerveSubsystem swerve;
  protected final CommandXboxController driverController = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  protected final AutoManager auto = new AutoManager();

  public RobotContainer() {
    try {
      this.layout = AprilTagFieldLayout.loadFromResource("/deploy/exampleAprilLayout.json");
      // Uses blue side as default in the event that the alliance color is null.
      final var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : DriverStation.Alliance.Blue;
      layout.setOrigin(alliance == DriverStation.Alliance.Blue ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
      this.swerve = new SwerveSubsystem(camera, layout);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    configureButtonBindings();

    if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
      PPLibTelemetry.enableCompetitionMode();
    }
  }

  private void configureButtonBindings() {
    driverController.a().onTrue(new ResetGyroCommand(swerve));
    driverController.b().onTrue(new SetXCommand(swerve));
    driverController.x().onTrue(new ApproachTargetCommand(camera, layout));
  }
}
