package br.com.megazord7563.subsystems.algaeintake;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;

public class AlgaeIntakeConstants {
  // The legacy robot used the roboRIO CAN bus. Confirm that these devices are now wired to the
  // CANivore before enabling the mechanism.
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  // TODO: Validate this provisional tolerance against the intake geometry and alignment needs.
  public static final double PIVOT_POSITION_TOLERANCE_RAD = Math.toRadians(2.0);

  public static final class PivotMotor {
    // Legacy OfficialCode-2025 ArticuladorAlgae motor ID.
    public static final int MOTOR_ID = 22;
    // Inverse of the legacy motor-to-output reduction: (1/12) * (42/96).
    public static final double MOTOR_GEAR_RATIO = 1.0 / ((1.0 / 12.0) * (42.0 / 96.0));
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    // Keep conservative provisional limits; the legacy Spark MAX limit is not directly
    // equivalent to Phoenix 6 stator/supply current limits.
    public static final Current STATOR_CURRENT_LIMIT =
        Amps.of(80); // TODO: Validate on the Kraken X60.
    public static final Current SUPPLY_CURRENT_LIMIT =
        Amps.of(30); // TODO: Validate for the power budget.
    public static final double MOTOR_INERTIA =
        0.2; // TODO: Replace simulation placeholder with measured inertia.
    public static final double MAX_VOLTAGE = 8.0; // TODO: Confirm a safe voltage limit.

    // Legacy gains were tuned for a NEO/Spark MAX position controller and are not portable to
    // this Kraken X60/Phoenix 6 position loop. Characterize and retune on the current mechanism.
    public static class FeedBackController {
      public static final double kP = 0.0; // TODO: Tune on hardware.
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static class SIM {
        public static final double kP = 0.0; // TODO: Tune the simulation model.
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    // TODO: Characterize motion feedforward and gravity compensation after defining geometry and
    // zero angle.
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

  public static final class RollerMotor {
    // Legacy OfficialCode-2025 algae roller motor ID.
    public static final int MOTOR_ID = 23;
    // Inverse of the legacy motor-to-wheel reduction: (1/3) * (41/31.84).
    public static final double MOTOR_GEAR_RATIO = 1.0 / ((1.0 / 3.0) * (41.0 / 31.84));
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    // Legacy Spark MAX current limits are not directly equivalent to Phoenix 6 current limits.
    public static final Current STATOR_CURRENT_LIMIT =
        Amps.of(80); // TODO: Validate on the Kraken X60.
    public static final Current SUPPLY_CURRENT_LIMIT =
        Amps.of(30); // TODO: Validate for the power budget.
    public static final double MOTOR_INERTIA =
        0.02; // TODO: Replace simulation placeholder with measured inertia.
    public static final double MAX_VOLTAGE = 8.0; // TODO: Confirm a safe voltage limit.

    // Legacy gains were tuned for a NEO/Spark MAX controller, not this Kraken X60 velocity loop.
    public static class FeedBackController {
      public static final double kP = 0.0; // TODO: Tune on hardware.
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static class SIM {
        public static final double kP = 0.0; // TODO: Tune the simulation model.
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    public static class FeedForwardController {
      public static final double kV = 0.0; // TODO: Characterize on the current motor and gearing.
      public static final double kS = 0.0;
      public static final double kA = 0.0;

      public static class SIM {
        public static final double kV = 0.0;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
      }
    }
  }
}
