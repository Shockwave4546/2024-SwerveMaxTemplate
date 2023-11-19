package frc.shuffleboard;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Wraps [SparkMaxPIDController] because it isn't natively supported by Shuffleboard.
 */
public class TunableSparkMaxPIDController implements Sendable {
  private final SparkMaxPIDController child;

  public TunableSparkMaxPIDController(SparkMaxPIDController child) {
    this.child = child;
  }

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
