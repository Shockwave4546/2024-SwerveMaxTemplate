package frc.robot.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * A class representing a boolean value on the Shuffleboard dashboard.
 */
public class ShuffleboardBoolean implements ShuffleboardValue {
  private static final boolean DEFAULT_VALUE = false;
  private final SimpleWidget widget;
  private final boolean def;

  /**
   * Constructs a new ShuffleboardBoolean object with the given parameters.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   * @param def  the default value for the widget
   */
  public ShuffleboardBoolean(ShuffleboardTab tab, String name, boolean def) {
    this.widget = tab.add(name, def);
    this.def = def;
    widget.withWidget(BuiltInWidgets.kToggleButton);
  }

  /**
   * Constructs a new ShuffleboardBoolean object with the given parameters. The default value for the widget is set to false.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   */
  public ShuffleboardBoolean(ShuffleboardTab tab, String name) {
    this(tab, name, DEFAULT_VALUE);
  }

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardBoolean object
   */
  public ShuffleboardBoolean withSize(int length, int height) {
    widget.withSize(length, height);
    return this;
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardBoolean object
   */
  public ShuffleboardBoolean withPosition(int x, int y) {
    widget.withPosition(x, y);
    return this;
  }

  /**
   * Retrieves the current value of the ShuffleboardBoolean object.
   *
   * @return the current value of the ShuffleboardBoolean object
   */
  public boolean get() {
    return widget.getEntry().getBoolean(def);
  }

  /**
   * Sets the value of the ShuffleboardBoolean object.
   *
   * @param value the new value to set
   */
  public void set(boolean value) {
    widget.getEntry().setBoolean(value);
  }

  /**
   * Returns the Raw GenericEntry object associated with this ShuffleboardBoolean.
   *
   * @return the Raw GenericEntry object
   */
  @Override public GenericEntry getRaw() {
    return widget.getEntry();
  }
}