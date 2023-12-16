package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveSubsystem;

public class ResetGyroCommand extends Command {
  private final SwerveSubsystem swerve;
  
  public ResetGyroCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    swerve.zeroHeading();
  }

  @Override public boolean isFinished() {
    return true;
  }
}
