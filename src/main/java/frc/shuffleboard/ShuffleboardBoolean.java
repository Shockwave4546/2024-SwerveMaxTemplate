package frc.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 *
 */
public class ShuffleboardBoolean implements ShuffleboardValue {
  private static final boolean DEFAULT_VALUE = false;
  private final SimpleWidget widget;
  private final boolean def;

  /**
   * @param tab
   * @param name
   * @param def
   */
  public ShuffleboardBoolean(ShuffleboardTab tab, String name, boolean def) {
    this.widget = tab.add(name, def);
    this.def = def;
    widget.withWidget(BuiltInWidgets.kToggleButton);
  }

  /**
   * @param tab
   * @param name
   */
  public ShuffleboardBoolean(ShuffleboardTab tab, String name) {
    this(tab, name, DEFAULT_VALUE);
  }

  /**
   * @param length
   * @param width
   * @return
   */
  public ShuffleboardBoolean withSize(int length, int width) {
    widget.withSize(length, width);
    return this;
  }

  /**
   * @param x
   * @param y
   * @return
   */
  public ShuffleboardBoolean withPosition(int x, int y) {
    widget.withPosition(x, y);
    return this;
  }

  /**
   * @return
   */
  public boolean get() {
    return widget.getEntry().getBoolean(def);
  }

  /**
   * @param value
   */
  public void set(boolean value) {
    widget.getEntry().setBoolean(value);
  }

  /* (non-Javadoc)
   * @see frc.robot.utils.shuffleboard.ShuffleboardValue#getRaw()
   */
  @Override public GenericEntry getRaw() {
    return widget.getEntry();
  }
}