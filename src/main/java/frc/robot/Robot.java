package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveDriveCommand;

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
    container.drive.setDefaultCommand(new SwerveDriveCommand(container.driverController, container.drive));
  }

  @Override public void teleopPeriodic() {

  }
}