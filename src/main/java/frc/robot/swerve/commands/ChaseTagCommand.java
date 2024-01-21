package frc.robot.swerve.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.pose.VisionSubsystem;
import frc.robot.swerve.SwerveSubsystem;

import static frc.robot.Constants.DriveConstants;

public class ChaseTagCommand extends Command {
  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(2, 0.0, 0.0), new Rotation3d());
  private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(DriveConstants.MAX_SPEED_METERS_PER_SECOND, 0));
  private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0.0, 0.0,  new TrapezoidProfile.Constraints(DriveConstants.MAX_SPEED_METERS_PER_SECOND, 0));
  private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0.0, 0.0,  new TrapezoidProfile.Constraints(DriveConstants.MAX_ANGULAR_SPEED, 4));
  private final VisionSubsystem vision;
  private final PoseEstimatorSubsystem poseEstimator;
  private final SwerveSubsystem swerve;

  public ChaseTagCommand(VisionSubsystem vision, PoseEstimatorSubsystem poseEstimator, SwerveSubsystem swerve) {
    this.vision = vision;
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;

    xController.setTolerance(0.2);

    yController.setTolerance(0.2);

    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(vision, poseEstimator, swerve);
  }

  @Override public void initialize() {
    xController.reset(poseEstimator.getPose2d().getX());
    yController.reset(poseEstimator.getPose2d().getY());
    omegaController.reset(poseEstimator.getPose2d().getRotation().getRadians());
  }

  @Override public void execute() {
    if (!vision.hasViableTarget()) {
      swerve.stop();
      return;
    }

    if (vision.getTag().getFiducialId() != TAG_TO_CHASE) return;
    final var tagPose = vision.getTagRelativeToCenterPose();
    final var goalPose = tagPose.transformBy(TAG_TO_GOAL).toPose2d();
    final var robotPose = poseEstimator.getPose2d();

    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    omegaController.setGoal(goalPose.getRotation().getRadians());

    final var xSpeed = xController.atGoal() ? 0 : -xController.calculate(robotPose.getX());
    final var ySpeed = yController.atGoal() ? 0 : -yController.calculate(robotPose.getY());
    final var rotSpeed = omegaController.atGoal() ? 0 : -omegaController.calculate(robotPose.getRotation().getRadians());

    swerve.drive(xSpeed, ySpeed, rotSpeed, true, true);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }
}
