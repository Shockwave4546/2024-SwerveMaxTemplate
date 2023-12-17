package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.commands.ResetGyroCommand;
import frc.robot.swerve.commands.SetXCommand;
import org.photonvision.PhotonCamera;

public class RobotContainer {
  protected final PhotonCamera camera = new PhotonCamera("OV9281");
  protected final SwerveSubsystem swerve = new SwerveSubsystem(camera);
  protected final CommandXboxController driverController = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  protected final AutoManager auto = new AutoManager();

  public RobotContainer() {
    configureButtonBindings();

    if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
      PPLibTelemetry.enableCompetitionMode();
    }
  }

  private void configureButtonBindings() {
    driverController.a().onTrue(new ResetGyroCommand(swerve));
    driverController.b().onTrue(new SetXCommand(swerve));
  }
}
