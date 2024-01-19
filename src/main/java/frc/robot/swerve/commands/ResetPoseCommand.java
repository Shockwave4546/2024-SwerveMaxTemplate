package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pose.PoseEstimatorSubsystem;

public class ResetPoseCommand extends Command {
  private final PoseEstimatorSubsystem poseEstimator;
  
  public ResetPoseCommand(PoseEstimatorSubsystem poseEstimator) {
    this.poseEstimator = poseEstimator;
    addRequirements(poseEstimator);
  }

  @Override public void initialize() {
    poseEstimator.zeroHeading();
  }

  @Override public boolean isFinished() {
    return true;
  }
}