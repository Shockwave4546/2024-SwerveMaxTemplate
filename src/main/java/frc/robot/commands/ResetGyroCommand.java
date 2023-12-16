package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroCommand extends Command {
  private final DriveSubsystem drive;
  
  public ResetGyroCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override public void initialize() {
    drive.zeroHeading();
  }

  @Override public boolean isFinished() {
    return true;
  }
}
