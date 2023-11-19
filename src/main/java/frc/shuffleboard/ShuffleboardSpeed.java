package frc.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardSpeed extends ShuffleboardDouble {
  public ShuffleboardSpeed(ShuffleboardTab tab, String name, double def) {
    super(tab, name, def);
    withMinMax(-1.0, 1.0);
  }

  public ShuffleboardSpeed(ShuffleboardTab tab, String name) {
    this(tab, name, DEFAULT_VALUE);
  }

  public ShuffleboardSpeed(String name, double def) {
    this(GlobalTab.DEBUG, name, def);
  }

  public ShuffleboardSpeed(String name) {
    this(GlobalTab.DEBUG, name, DEFAULT_VALUE);
  }
}