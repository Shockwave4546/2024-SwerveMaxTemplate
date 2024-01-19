package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.swerve.SwerveSubsystem;

import static frc.robot.Constants.*;

public class AutoManager {
  /**
   * Automatically generates all the autonomous modes from ../pathplanner/autos
   */
  private final SendableChooser<Command> chooser;

  /**
   * Registers all Commands into NamedCommands.
   */
  public AutoManager(SwerveSubsystem swerve, PoseEstimatorSubsystem poseEstimator) {
    AutoBuilder.configureHolonomic(
            poseEstimator::getPose2d,
            poseEstimator::resetOdometry,
            swerve::getRelativeChassisSpeed,
            swerve::driveAutonomous,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(AutoConstants.DRIVING_P, AutoConstants.DRIVING_I, AutoConstants.DRIVING_D),
                    new PIDConstants(AutoConstants.TURNING_P, AutoConstants.TURNING_I, AutoConstants.TURNING_D),
                    DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                    DriveConstants.WHEEL_BASE / 2,
                    new ReplanningConfig()
            ),
            this::shouldFlipPath,
            swerve
    );

    this.chooser = AutoBuilder.buildAutoChooser("Do nothing.");
    Tabs.MATCH.add("Autonomous", chooser).withSize(3, 2).withPosition(3, 0);
  }

  /**
   * Red = flip, since PathPlanner uses blue as the default wall.
   *
   * @return whether the autonomous path should be flipped dependent on the alliance color.
   */
  private boolean shouldFlipPath() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  /**
   * Schedules the selected autonomous mode.
   */
  public void executeRoutine() {
    chooser.getSelected().schedule();
  }
}