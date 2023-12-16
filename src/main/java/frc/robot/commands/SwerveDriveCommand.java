package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.shuffleboard.ShuffleboardBoolean;
import frc.shuffleboard.ShuffleboardDouble;

public class SwerveDriveCommand extends Command {
  private final ShuffleboardBoolean isFieldRelative = new ShuffleboardBoolean(Constants.Tabs.MATCH, "Is Field Relative?", true);
  private final CommandXboxController controller;
  private final DriveSubsystem drive;

  public SwerveDriveCommand(CommandXboxController controller, DriveSubsystem drive) {
    this.controller = controller;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override public void execute() {
    drive.drive(
            MathUtil.applyDeadband(controller.getLeftY(), IOConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(controller.getLeftX(), IOConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(controller.getRightX(), IOConstants.DRIVE_DEADBAND),
            isFieldRelative.get()
    );
  }
}
