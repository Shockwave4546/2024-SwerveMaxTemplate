package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import java.util.List;

public class AutoManager {
  /**
   * Automatically generates all the autonomous modes from ../pathplanner/autos
   */
  private final SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser("Do nothing.");

  /**
   * Registers all Commands into NamedCommands.
   */
  public AutoManager() {
    NamedCommands.registerCommand("Print", new PrintCommand("Hello world!!!"));
    Constants.Tabs.MATCH.add("Autonomous", chooser).withSize(3, 2);
  }

  /**
   * Schedules the selected autonomous mode.
   */
  public void executeRoutine() {
    chooser.getSelected().schedule();
  }
}