package frc.robot.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.Map;

/**
 * A class representing a double value on the Shuffleboard dashboard.
 */
public class ShuffleboardDouble implements ShuffleboardValue {
  protected static final double DEFAULT_VALUE = 0.0;
  private final SimpleWidget widget;
  private final double def;

  /**
   * Constructs a new ShuffleboardDouble object with the given parameters.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   * @param def  the default value for the widget
   */
  public ShuffleboardDouble(ShuffleboardTab tab, String name, double def) {
    this.widget = tab.add(name, def);
    this.def = def;
  }

  /**
   * Constructs a new ShuffleboardDouble object with the given parameters. The default value for the widget is set to 0.0.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   */
  public ShuffleboardDouble(ShuffleboardTab tab, String name) {
    this(tab, name, DEFAULT_VALUE);
  }

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardDouble object
   */
  public ShuffleboardDouble withSize(int length, int height) {
    widget.withSize(length, height);
    return this;
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardDouble object
   */
  public ShuffleboardDouble withPosition(int x, int y) {
    widget.withPosition(x, y);
    return this;
  }

  /**
   * Sets the minimum and maximum values for the widget.
   *
   * @param min the minimum value for the widget
   * @param max the maximum value for the widget
   * @return the modified ShuffleboardDouble object
   */
  @SuppressWarnings("UnusedReturnValue")
  public ShuffleboardDouble withMinMax(double min, double max) {
    widget.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", String.valueOf(min), "max", String.valueOf(max)));
    return this;
  }

  /**
   * Retrieves the current value of the ShuffleboardDouble object.
   *
   * @return the current value of the ShuffleboardDouble object
   */
  public double get() {
    return widget.getEntry().getDouble(def);
  }

  /**
   * Sets the value of the ShuffleboardDouble object.
   *
   * @param value the new value to set
   */
  public void set(double value) {
    widget.getEntry().setDouble(value);
  }

  /**
   * Returns the Raw GenericEntry object associated with this ShuffleboardDouble.
   *
   * @return the Raw GenericEntry object
   */
  @Override public GenericEntry getRaw() {
    return widget.getEntry();
  }
}