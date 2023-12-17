package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.commands.SwerveDriveCommand;

public class Robot extends TimedRobot {
  private RobotContainer container;

  @Override public void robotInit() {
    container = new RobotContainer();
  }

  @Override public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override public void autonomousInit() {
    container.auto.executeRoutine();
  }

  @Override public void autonomousPeriodic() {

  }

  @Override public void teleopInit() {
    container.swerve.setDefaultCommand(new SwerveDriveCommand(container.driverController, container.swerve));
  }

  @Override public void teleopExit() {
    // For debugging purposes only.
    container.swerve.resetEncoders();
    container.swerve.zeroHeading();
    container.swerve.resetOdometry(new Pose2d());
  }

  @Override public void teleopPeriodic() {

  }
}
