package frc.robot.swerve.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveSubsystem;

public class SetXCommand extends Command {
  private final SwerveSubsystem swerve;

  public SetXCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  @Override public void initialize() {
    setX();
  }

  @Override public boolean isFinished() {
    return true;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  private void setX() {
    swerve.setModuleStates(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    );
  }
}