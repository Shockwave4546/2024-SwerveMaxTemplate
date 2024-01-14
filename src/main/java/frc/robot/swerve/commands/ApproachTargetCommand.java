package frc.robot.swerve.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;

public class ApproachTargetCommand extends Command {
  private final PhotonCamera camera;
  private final AprilTagFieldLayout layout;

  public ApproachTargetCommand(PhotonCamera camera, AprilTagFieldLayout layout) {
    this.camera = camera;
    this.layout = layout;
  }

  @Override public void initialize() {
    final var result = camera.getLatestResult();
    if (!result.hasTargets()) return;
    final var target = result.getBestTarget();
    final var targetPose = layout.getTagPose(target.getFiducialId());
    if (targetPose.isEmpty()) return;
    final var command = AutoBuilder.pathfindToPose(
            targetPose.get().toPose2d().plus(new Transform2d(-1, -1, new Rotation2d())),
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI));
    command.schedule();
  }
}