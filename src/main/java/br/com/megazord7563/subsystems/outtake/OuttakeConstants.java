// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.outtake;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Mass;

public class OuttakeConstants {
  public static final String OUTTAKE_MECHANISM_NAME = "Outtake";
  public static final Mass OUTTAKE_MASS = Kilograms.of(3);

  // The legacy robot used the roboRIO CAN bus. Confirm that these devices are now wired to the
  // CANivore before enabling the mechanism.
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  public static final class OuttakeMotor {
    // Legacy OfficialCode-2025 Coral motor ID.
    public static final int MOTOR_ID = 20;
    // Legacy coral roller used a 1:1 motor-to-mechanism reduction.
    public static final double MOTOR_GEAR_RATIO = 1.0;
    // The legacy TalonFX was configured as Clockwise_Positive.
    public static final boolean MOTOR_INVERTED = true;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT =
        Amps.of(80); // TODO: Validate on the Kraken X60.
    public static final Current SUPPLY_CURRENT_LIMIT =
        Amps.of(30); // TODO: Validate for the power budget.

    public static final double MOTOR_INERTIA =
        0.2; // TODO: Measure or calculate the mechanism-side moment of inertia.
    public static final double MAX_VOLTAGE =
        8.0; // TODO: Reconcile with the legacy +/-2.65 V limit and validate on hardware.

    // The legacy Coral TalonFX used a position-oriented control path. The current Outtake uses
    // VelocityVoltage, so its old position-loop gains must not be copied into this velocity loop.
    public static class FeedBackController {
      public static final double kP = 0.0; // TODO: Characterize and tune the velocity loop.
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

  public static final class PivotMotor {
    // Legacy OfficialCode-2025 ArticuladorCoral motor ID.
    public static final int MOTOR_ID = 21;
    // Inverse of the legacy motor-to-output reduction:
    // (1/3) * (20/68) * (20/42).
    public static final double MOTOR_GEAR_RATIO =
        1.0 / ((1.0 / 3.0) * (20.0 / 68.0) * (20.0 / 42.0));
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT =
        Amps.of(80); // TODO: Validate on the Kraken X60.
    public static final Current SUPPLY_CURRENT_LIMIT =
        Amps.of(30); // TODO: Validate for the power budget.

    public static final double MOTOR_INERTIA =
        0.4; // TODO: Measure or calculate the pivot-side moment of inertia.
    public static final double MAX_VOLTAGE = 8.0; // TODO: Confirm a safe voltage limit.
    public static final double POSITION_TOLERANCE_RAD =
        0.2; // TODO: Tune against the pivot geometry.

    // Legacy gains were tuned for a NEO/Spark MAX MAXMotion controller and are not portable
    // to this Kraken X60/Phoenix 6 position loop.
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

    // TODO: Add characterized gravity feedforward after validating the pivot zero angle.
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
