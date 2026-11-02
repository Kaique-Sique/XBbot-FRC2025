package br.com.megazord7563.subsystems.ramp;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;

public class RampConstants {
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  // TODO: Validate this provisional tolerance against the ramp geometry and required alignment.
  public static final double POSITION_TOLERANCE_RAD = Math.toRadians(2.0);

  public static final class Motor {
    public static final int MOTOR_ID = 0; // TODO: Set a unique CAN ID before enabling this motor.
    public static final double MOTOR_GEAR_RATIO = 1.0; // TODO: Replace with the measured mechanism reduction before hardware operation.
    public static final boolean MOTOR_INVERTED = false; // TODO: Verify motor direction on the robot.
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80); // TODO: Validate against the motor and mechanism.
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30); // TODO: Validate against the power budget.

    public static final double MOTOR_INERTIA = 0.2; // TODO: Replace this simulation placeholder with measured mechanism inertia.
    public static final double MAX_VOLTAGE = 8.0; // TODO: Confirm a safe voltage limit for the mechanism.

    // TODO: Characterize and tune these gains before enabling closed-loop position control.
    public static class FeedBackController {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static class SIM {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    // TODO: Characterize motion feedforward and gravity compensation after defining the ramp geometry and zero angle.
    public static class FeedForwardController {
      public static final double kV = 0.0;
      public static final double kS = 0.0;
      public static final double kA = 0.0;
      public static final double kG = 0.0;

      public static class SIM {
        public static final double kV = 0.0;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
      }
    }
  }
}
