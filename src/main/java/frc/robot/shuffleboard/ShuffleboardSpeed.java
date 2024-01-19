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

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardDouble object
   */
  public ShuffleboardSpeed withSize(int length, int height) {
    super.withSize(length, height);
    return this;
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardDouble object
   */
  public ShuffleboardSpeed withPosition(int x, int y) {
    super.withPosition(x, y);
    return this;
  }
}