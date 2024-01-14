package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A Shuffleboard control for adjusting the speed value.
 */
public class ShuffleboardSpeed extends ShuffleboardDouble {
  /**
   * Creates a ShuffleboardSpeed widget with the given name and default value,
   * and adds it to the specified Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the widget to
   * @param name the name of the ShuffleboardSpeed widget
   * @param def the default value of the ShuffleboardSpeed widget
   */
  public ShuffleboardSpeed(ShuffleboardTab tab, String name, double def) {
    super(tab, name, def);
    withMinMax(-1.0, 1.0);
  }

  /**
   * Creates a ShuffleboardSpeed widget with the given name and a default value,
   * and adds it to the specified Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the widget to
   * @param name the name of the ShuffleboardSpeed widget
   */
  public ShuffleboardSpeed(ShuffleboardTab tab, String name) {
    this(tab, name, DEFAULT_VALUE);
  }
}