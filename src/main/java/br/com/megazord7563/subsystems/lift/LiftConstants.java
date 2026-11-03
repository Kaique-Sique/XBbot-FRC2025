package br.com.megazord7563.subsystems.lift;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the Lift subsystem. */
public final class LiftConstants {
  // The CAN bus follows the existing mechanism convention. Confirm wiring before enabling Lift.
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  // TODO: Replace the sentinel IDs with the confirmed CAN IDs before using the real mechanism.
  public static final int LEADER_MOTOR_ID = -1;
  public static final int FOLLOWER_MOTOR_ID = -1;

  // The follower is mechanically mirrored relative to the leader.
  public static final boolean FOLLOWER_OPPOSES_LEADER = true;

  // TODO: Define the actual motor-to-elevator reduction and elevator geometry.
  // This placeholder only allows the simulation model to be constructed; it is not a validated ratio.
  public static final double MOTOR_GEAR_RATIO = 1.0;

  // Both Kraken X60 motors contribute torque to the same elevator mechanism.
  public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(2);

  // TODO: Confirm safe voltage, current limits, mechanical travel, and the elevator's load.
  public static final double MAX_VOLTAGE = 12.0;

  // TODO: Characterize the Lift controller gains on the physical mechanism.
  public static final class FeedBackController {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final class SIM {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

  }

  // TODO: Characterize motion feedforward after the elevator geometry and mass are known.
  public static final class FeedForwardController {
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    public static final class SIM {
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;
      public static final double kG = 0.0;
    }

  }

  // TODO: Replace with measured or calculated mechanism inertia.
  public static final double MOTOR_INERTIA = 1.0;

  // TODO: Set a tolerance appropriate for the elevator's scoring and safety requirements.
  public static final double POSITION_TOLERANCE_RAD = Math.toRadians(2.0);

  private LiftConstants() {}
}
