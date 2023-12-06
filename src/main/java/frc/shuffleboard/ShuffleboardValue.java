package frc.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;

/**
 * The ShuffleboardValue interface represents a shuffleboard value that can be displayed on the Shuffleboard dashboard.
 */
@SuppressWarnings("unused")
public interface ShuffleboardValue {
  /**
   * Retrieves the raw GenericEntry object.
   *
   * @return The raw GenericEntry object.
   */
  GenericEntry getRaw();
}