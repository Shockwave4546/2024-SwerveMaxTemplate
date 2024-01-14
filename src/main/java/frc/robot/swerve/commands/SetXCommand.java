package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveSubsystem;

public class SetXCommand extends Command {
  private final SwerveSubsystem swerve;

  public SetXCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    swerve.toggleX();
  }

  @Override public boolean isFinished() {
    return true;
  }
}