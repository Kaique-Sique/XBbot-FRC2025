package br.com.megazord7563.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;

/** Hardware and control constants for the Climber subsystem. */
public final class ClimberConstants {
  // The current robot uses the CAN bus selected by the generated swerve configuration.
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  public static final class Motor {
    // The legacy EscaladorSubsystem leader used CAN ID 19.
    // The legacy follower (CAN ID 18) is intentionally not instantiated: this design has one motor.
    public static final int MOTOR_ID = 19;
    public static final boolean MOTOR_INVERTED = false;
    // The legacy TalonFX configuration used a sensor-to-mechanism ratio of 1.
    public static final double MOTOR_GEAR_RATIO = 1.0;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    // Values carried over from the legacy TalonFX configuration.
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
    public static final double MAX_VOLTAGE = 10.0;

    public static final double VELOCITY_KS = 0.1;
    public static final double VELOCITY_KV = 0.12;
    public static final double VELOCITY_KP = 0.0665;
    public static final double VELOCITY_KI = 0.0;
    public static final double VELOCITY_KD = 0.0;

    public static final double POSITION_KP = 2.4;
    public static final double POSITION_KI = 0.0;
    public static final double POSITION_KD = 0.0;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 5.0;
    public static final double MOTION_MAGIC_ACCELERATION = 10.0;
    public static final double MOTION_MAGIC_JERK = 100.0;
    public static final double CLOSED_LOOP_RAMP_PERIOD = 0.25;

    // TODO: Validate the legacy controller gains and current limits on the single-motor mechanism.
    // TODO: Confirm the mechanical reduction, safe travel limits, and homing/zeroing procedure.
    // TODO: Replace the simulation inertia placeholder after measuring or estimating the mechanism.
    public static final double SIM_MOTOR_INERTIA = 0.02;
    public static final double SIM_POSITION_KP = 0.0;
    public static final double SIM_POSITION_KI = 0.0;
    public static final double SIM_POSITION_KD = 0.0;
    public static final double SIM_VELOCITY_KP = 0.0;
    public static final double SIM_VELOCITY_KI = 0.0;
    public static final double SIM_VELOCITY_KD = 0.0;
  }

  private ClimberConstants() {}
}
