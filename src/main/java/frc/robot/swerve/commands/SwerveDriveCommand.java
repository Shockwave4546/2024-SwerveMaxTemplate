package frc.robot.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.SwerveSubsystem;

import static frc.robot.Constants.IO;

public class SwerveDriveCommand extends Command {
  private final CommandXboxController controller;
  private final SwerveSubsystem swerve;

  public SwerveDriveCommand(CommandXboxController controller, SwerveSubsystem drive) {
    this.controller = controller;
    this.swerve = drive;
    addRequirements(drive);
  }

  @Override public void execute() {
    swerve.drive(
            -MathUtil.applyDeadband(controller.getLeftY(), IO.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(controller.getLeftX(), IO.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(controller.getRightX(), IO.DRIVE_DEADBAND),
            swerve.isFieldRelative(),
            false
    );
  }
}
