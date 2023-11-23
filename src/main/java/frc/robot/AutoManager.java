package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoManager {
  /**
   * Automatically generates all the autonomous modes from ../pathplanner/autos
   */
  private final SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();

  /**
   * Registers all Commands into NamedCommands.
   */
  public AutoManager() {
    NamedCommands.registerCommand("Print", new PrintCommand("Hello world!!!"));
    Constants.Tabs.MATCH.add("Autonomous", chooser).withSize(3, 2);
  }

  /**
   * Runs the selected autonomous mode.
   */
  public void executeRoutine() {
    chooser.getSelected().execute();
  }
}