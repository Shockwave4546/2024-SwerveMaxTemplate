package frc.robot.simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class FieldSim {
  private final DriveSubsystem drive;
  private final Field2d field = new Field2d();
  private final Map<ModulePosition, Pose2d> swerveModulePoses = new LinkedHashMap<>(4) {{
    put(ModulePosition.FRONT_LEFT, new Pose2d());
    put(ModulePosition.FRONT_RIGHT, new Pose2d());
    put(ModulePosition.BACK_LEFT, new Pose2d());
    put(ModulePosition.BACK_RIGHT, new Pose2d());
  }};

  public FieldSim(DriveSubsystem drive) {
    this.drive = drive;
    Constants.Tabs.MATCH.add("Field Sim", field);
  }

  public void updatePoses() {
    field.setRobotPose(drive.getPose());

    for (final var position : swerveModulePoses.keySet()) {
      final var updatedTranslation = DriveConstants.MODULE_TRANSLATIONS.get(position)
              .rotateBy(drive.getPose().getRotation())
              .plus(drive.getPose().getTranslation());

      final var updatedRotation = drive.getModule(position).getPosition().angle.plus(drive.getHeadingRotation2d());
      swerveModulePoses.put(position, new Pose2d(updatedTranslation, updatedRotation));

      field.getObject("Swerve Modules").setPoses(swerveModulePoses.values().toArray(Pose2d[]::new));
    }
  }
}