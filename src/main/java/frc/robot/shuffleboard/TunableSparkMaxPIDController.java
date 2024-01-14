package frc.robot.shuffleboard;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Wraps [SparkMaxPIDController] because it isn't natively supported by Shuffleboard.
 */
public class TunableSparkMaxPIDController implements Sendable {
  private final SparkPIDController child;

  /**
   * Constructs a new instance of the TunableSparkMaxPIDController with the specified child controller.
   *
   * @param child the child controller to use for control operations
   */
  public TunableSparkMaxPIDController(SparkPIDController child) {
    this.child = child;
  }

  /**
   * Initializes the Sendable interface for the TunableSparkMaxPIDController.
   *
   * @param builder the Sendable builder used to configure the SmartDashboard properties
   */
  @Override public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", child::getP, child::setP);
    builder.addDoubleProperty("i", child::getI, child::setI);
    builder.addDoubleProperty("d", child::getD, child::setD);
    builder.addDoubleProperty("f", child::getFF, child::setFF);

    // Ignore these two fields, but it's necessary to satisfy the PIDController SmartDashboard type.
    builder.addDoubleProperty("setpoint", null, null);
    builder.addBooleanProperty("enabled", () -> true, null);
  }
}
