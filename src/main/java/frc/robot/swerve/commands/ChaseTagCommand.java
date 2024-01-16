package frc.robot.swerve.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.swerve.SwerveSubsystem;
import org.photonvision.PhotonCamera;

public class ChaseTagCommand extends Command {
  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(2, 0.0, 0.0), new Rotation3d());
  private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(3, 2));
  private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0.0, 0.0,  new TrapezoidProfile.Constraints(3, 2));
  private final ProfiledPIDController omegaController = new ProfiledPIDController(1.5, 0.0, 0.0,  new TrapezoidProfile.Constraints(8, 8));
  private final PhotonCamera camera;
  private final SwerveSubsystem swerve;

  public ChaseTagCommand(PhotonCamera camera, SwerveSubsystem swerve) {
    this.camera = camera;
    this.swerve = swerve;

    xController.setTolerance(0.2);

    yController.setTolerance(0.2);

    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(0, 2 * Math.PI);


    addRequirements(swerve);
  }

  @Override public void initialize() {
    xController.reset(swerve.getPose().getX());
    yController.reset(swerve.getPose().getY());
    omegaController.reset(swerve.getPose().getRotation().getRadians());
  }

  @Override public void execute() {
    System.out.println("Ahhhh");
    final var robotPose = new Pose3d(
            swerve.getPose().getX(),
            swerve.getPose().getY(),
            0.0,
            new Rotation3d(0.0, 0.0, swerve.getPose().getRotation().getRadians())
    );

    final var result = camera.getLatestResult();
    if (!result.hasTargets()) return;
    final var target = result.getBestTarget();
    if (target.getFiducialId() != TAG_TO_CHASE) return;
    if (target.getPoseAmbiguity() > 0.2) return;
    System.out.println("Good");
    // Transform the robot's pose to find the camera's pose
    final var cameraPose = robotPose.transformBy(Constants.DriveConstants.ROBOT_TO_CAMERA);
     // Transform the camera's pose to the target's pose.
    final var camToTarget = target.getBestCameraToTarget().plus(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, Math.PI)));
    final var targetPose = cameraPose.transformBy(camToTarget);
    // Transform the tag's pose to set our goal.
    final var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    omegaController.setGoal(goalPose.getRotation().getRadians());

    final var xSpeed = xController.atGoal() ? 0 : -xController.calculate(swerve.getPose().getX());
    final var ySpeed = yController.atGoal() ? 0 : -yController.calculate(swerve.getPose().getY());
    final var rotSpeed = omegaController.atGoal() ? 0 : -omegaController.calculate(swerve.getPose().getRotation().getRadians());

    System.out.println("Robot pose degrees: " + robotPose.getRotation().toRotation2d().getDegrees());
    System.out.println("Goal pose degrees: " + goalPose.getRotation().getDegrees());

    swerve.drive(
            xSpeed,
            ySpeed,
            rotSpeed,
            true
    );
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }
}
