package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  protected final DriveSubsystem drive = new DriveSubsystem();
  protected final CommandXboxController driverController = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  protected final AutoManager auto = new AutoManager();

  public RobotContainer() {
    configureButtonBindings();

    if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
      PPLibTelemetry.enableCompetitionMode();
    }
  }

  private void configureButtonBindings() {
    driverController.a().onTrue(new RunCommand(drive::zeroHeading, drive));
  }
}
