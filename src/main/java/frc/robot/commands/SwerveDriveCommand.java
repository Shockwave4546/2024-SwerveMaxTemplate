package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.shuffleboard.GlobalTab;
import frc.shuffleboard.ShuffleboardSpeed;

public class SwerveDriveCommand extends Command {
  private final CommandXboxController controller;
  private final DriveSubsystem drive;

  public SwerveDriveCommand(CommandXboxController controller, DriveSubsystem drive) {
    this.controller = controller;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override public void execute() {
    // TODO: 11/5/2023 Don't use rate limiting for now (leads to weird feeling control).  
    drive.drive(
            MathUtil.applyDeadband(controller.getLeftY() * RobotContainer.SPEED_MULT.get(), IOConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(controller.getLeftX() * RobotContainer.SPEED_MULT.get(), IOConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(controller.getRightX(), IOConstants.DRIVE_DEADBAND),
            true,
            false
    );
  }
}
