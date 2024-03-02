package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.swerve.SwerveSubsystem;

public class SetSpeedMaxCommand extends Command {
  private final SwerveSubsystem swerve;
  private final double driveMax;
  private final double rotMax;

  public SetSpeedMaxCommand(SwerveSubsystem swerve, double driveMax, double rotMax) {
    this.swerve = swerve;
    this.driveMax = driveMax;
    this.rotMax = rotMax;
  }

  @Override public void execute() {
    swerve.setMaxSpeed(driveMax, rotMax);
  }

  @Override public void end(boolean interrupted) {
    swerve.setMaxSpeed(Constants.Swerve.DEFAULT_DRIVE_SPEED_MULTIPLIER, Constants.Swerve.DEFAULT_ROT_SPEED_MULTIPLIER);
  }
}