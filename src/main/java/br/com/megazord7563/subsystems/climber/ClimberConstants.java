package br.com.megazord7563.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;

public class ClimberConstants {
  // The legacy robot used the roboRIO CAN bus. Confirm that this device is now wired to the
  // CANivore before enabling the mechanism.
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  public static final class ClimberMotor {
    // The legacy EscaladorSubsystem leader used CAN ID 19.
    // The legacy follower (CAN ID 18) is intentionally not instantiated: this design has one motor.
    public static final int MOTOR_ID = 19;
    public static final double MOTOR_GEAR_RATIO = 1.0; // TODO: Confirm the actual climber reduction.
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(120); // TODO: Validate on the single-motor climber.
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70); // TODO: Validate for the robot power budget.
    public static final double MOTOR_INERTIA = 0.02; // TODO: Replace with measured or calculated mechanism inertia.
    public static final double MAX_VOLTAGE = 10.0; // TODO: Validate the safe operating voltage.
    public static final double CLOSED_LOOP_RAMP_PERIOD = 0.25;

    public static class PositionController {
      // Legacy Phoenix 6 Motion Magic position gains.
      public static final double kP = 2.4;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static class SIM {
        public static final double kP = 0.0; // TODO: Tune the simulation controller.
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    public static class FeedBackController {
      // Legacy Phoenix 6 velocity gains; validate after confirming the current mechanism ratio.
      public static final double kP = 0.0665;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static class SIM {
        public static final double kP = 0.0; // TODO: Tune the simulation controller.
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    public static class FeedForwardController {
      // Legacy velocity feedforward values.
      public static final double kS = 0.1;
      public static final double kV = 0.12;
      public static final double kA = 0.0; // TODO: Characterize acceleration feedforward.

      public static class SIM {
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
      }
    }

    public static class MotionMagic {
      public static final double CRUISE_VELOCITY = 5.0;
      public static final double ACCELERATION = 10.0;
      public static final double JERK = 100.0;
    }

    // TODO: Define safe climber travel limits and the homing/zeroing procedure.
    // TODO: Validate legacy gains and current limits on the single-motor mechanism.
  }

  private ClimberConstants() {}
}
