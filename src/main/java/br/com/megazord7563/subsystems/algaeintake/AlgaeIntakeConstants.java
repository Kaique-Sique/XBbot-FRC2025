package br.com.megazord7563.subsystems.algaeintake;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;

public class AlgaeIntakeConstants {
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  // TODO: Validate this provisional tolerance against the intake geometry and alignment needs.
  public static final double PIVOT_POSITION_TOLERANCE_RAD = Math.toRadians(2.0);

  public static final class PivotMotor {
    public static final int MOTOR_ID = 0; // TODO: Assign a unique CAN ID before enabling hardware.
    public static final double MOTOR_GEAR_RATIO = 1.0; // TODO: Replace with the measured mechanism reduction.
    public static final boolean MOTOR_INVERTED = false; // TODO: Verify motor direction on the robot.
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80); // TODO: Validate for the mechanism.
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30); // TODO: Validate for the power budget.
    public static final double MOTOR_INERTIA = 0.2; // TODO: Replace simulation placeholder with measured inertia.
    public static final double MAX_VOLTAGE = 8.0; // TODO: Confirm a safe voltage limit.

    // TODO: Characterize and tune before enabling closed-loop position control.
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

    // TODO: Characterize motion feedforward and gravity compensation after defining geometry and zero angle.
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
    public static final int MOTOR_ID = 0; // TODO: Assign a unique CAN ID before enabling hardware.
    public static final double MOTOR_GEAR_RATIO = 1.0; // TODO: Replace with the measured wheel reduction.
    public static final boolean MOTOR_INVERTED = false; // TODO: Verify collection direction on the robot.
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80); // TODO: Validate for the mechanism.
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30); // TODO: Validate for the power budget.
    public static final double MOTOR_INERTIA = 0.02; // TODO: Replace simulation placeholder with measured inertia.
    public static final double MAX_VOLTAGE = 8.0; // TODO: Confirm a safe voltage limit.

    // TODO: Characterize and tune before enabling closed-loop velocity control.
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

    public static class FeedForwardController {
      public static final double kV = 0.0;
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
