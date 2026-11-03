// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

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
    public static final double MOTOR_GEAR_RATIO =
        1.0; // TODO: Confirm the actual climber reduction.
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT =
        Amps.of(120); // TODO: Validate on the single-motor climber.
    public static final Current SUPPLY_CURRENT_LIMIT =
        Amps.of(80); // TODO: Validate for the robot power budget.
    public static final double MOTOR_INERTIA =
        1; // TODO: Replace with measured or calculated mechanism inertia.
    public static final double MAX_VOLTAGE = 12.0; // TODO: Validate the safe operating voltage.
    public static final double CLOSED_LOOP_RAMP_PERIOD = 0.25;

    public static final double POSITION_TOLERANCE_RAD =
        Math.toRadians(2.0); // TODO: Validate against the climber's mechanical requirements.

    public static class FeedBackController {
      public static final double kP = 0.0665;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static class SIM {
        // Convert the legacy TalonFX gains from volts per mechanism rotation to volts per radian.
        public static final double kP = FeedBackController.kP / (2.0 * Math.PI);
        public static final double kI = FeedBackController.kI / (2.0 * Math.PI);
        public static final double kD = FeedBackController.kD / (2.0 * Math.PI);
      }
    }

    public static class FeedForwardController {
      // Legacy TalonFX feedforward values use mechanism rotations as the angular unit.
      public static final double kS = 0.1;
      public static final double kV = 0.12;
      public static final double kA = 0.0; // TODO: Characterize acceleration feedforward.

      public static class SIM {
        // Convert velocity and acceleration gains from per rotation to per radian.
        public static final double kS = FeedForwardController.kS;
        public static final double kV = FeedForwardController.kV / (2.0 * Math.PI);
        public static final double kA = FeedForwardController.kA / (2.0 * Math.PI);
      }
    }

    public static class MotionMagic {
      // Phoenix values are mechanism rotations per second and rotations per second squared.
      public static final double CRUISE_VELOCITY = 5.0;
      public static final double ACCELERATION = 10.0;
      public static final double JERK = 100.0;
    }

    // TODO: Define safe mechanical travel limits and configure TalonFX software limits.
    // TODO: Define and implement a homing/zeroing procedure after selecting the reference sensor.
    // TODO: Confirm the CAN bus, gear ratio, motor inversion, inertia, current limits, voltage,
    // and legacy gains on the physical single-motor climber before enabling it.
  }

  private ClimberConstants() {}
}
