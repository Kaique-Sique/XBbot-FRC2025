// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.ramp;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;

public class RampConstants {
  // The legacy robot used the roboRIO CAN bus. Confirm that this device is now wired to the
  // CANivore before enabling the mechanism.
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  // TODO: Validate this provisional tolerance against the ramp geometry and required alignment.
  public static final double POSITION_TOLERANCE_RAD = Math.toRadians(2.0);

  public static final class Motor {
    // Legacy OfficialCode-2025 Ramp motor ID.
    public static final int MOTOR_ID = 17;
    // Inverse of the legacy motor-to-output reduction: (1/9) * (20/42).
    public static final double MOTOR_GEAR_RATIO = 1.0 / ((1.0 / 9.0) * (20.0 / 42.0));
    // The legacy Ramp Spark MAX was configured as inverted.
    public static final boolean MOTOR_INVERTED = true;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    // Legacy Spark MAX current limits are not directly equivalent to Phoenix 6 current limits.
    public static final Current STATOR_CURRENT_LIMIT =
        Amps.of(80); // TODO: Validate on the Kraken X60.
    public static final Current SUPPLY_CURRENT_LIMIT =
        Amps.of(30); // TODO: Validate for the power budget.

    public static final double MOTOR_INERTIA =
        0.2; // TODO: Replace this simulation placeholder with measured mechanism inertia.
    public static final double MAX_VOLTAGE =
        8.0; // TODO: Confirm a safe voltage limit for the mechanism.

    // Legacy gains (Spark MAX MAXMotion position control, in encoder-position units) are not
    // portable to this Kraken X60/Phoenix 6 position loop. Retune after validating the mechanism.
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

    // TODO: Characterize motion feedforward and gravity compensation after defining ramp geometry
    // and zero angle.
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
