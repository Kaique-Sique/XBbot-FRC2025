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

/** Hardware, control, and physics constants for the coral Outtake. */
public final class OuttakeConstants {
  private OuttakeConstants() {}

  public static final String OUTTAKE_MECHANISM_NAME = "Outtake";
  public static final Mass OUTTAKE_MASS = Kilograms.of(3);
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  /** Provisional mechanical values; validate against the assembled mechanism before enabling. */
  public static final class PivotGeometry {
    private PivotGeometry() {}

    public static final double ARM_LENGTH_METERS = 0.30;
    public static final double PAYLOAD_MASS_KG = 5.0;
    public static final double PAYLOAD_CENTER_OF_MASS_RADIUS_METERS = 0.25;
    public static final double ARM_MASS_KG = 1.0;
    public static final double ARM_CENTER_OF_MASS_RADIUS_METERS = ARM_LENGTH_METERS / 2.0;

    // Point-mass payload plus a uniform arm approximation.
    public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED =
        PAYLOAD_MASS_KG * Math.pow(PAYLOAD_CENTER_OF_MASS_RADIUS_METERS, 2)
            + ARM_MASS_KG * Math.pow(ARM_LENGTH_METERS, 2) / 3.0;

    // Angles are measured from horizontal. These are simulation stops until the real zero
    // reference and hard-stop geometry are confirmed.
    public static final double MIN_ANGLE_RAD = Math.toRadians(-20.0);
    public static final double MAX_ANGLE_RAD = Math.toRadians(110.0);
    public static final double STARTING_ANGLE_RAD = 0.0;
  }

  public static final class OuttakeMotor {
    private OuttakeMotor() {}

    // Provisional CAN ID, selected with permission to assign IDs for this implementation.
    public static final int MOTOR_ID = 24;
    public static final double MOTOR_GEAR_RATIO = 1.0;
    public static final boolean MOTOR_INVERTED = true;

    // WPILib does not expose a dedicated Kraken X44 model. This X60 FOC model is a documented
    // approximation for simulation only; the physical controller remains configured for the X44.
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
    public static final double MOTOR_INERTIA = 0.001;
    public static final double MAX_VOLTAGE = 8.0;

    public static final class FeedBackController {
      private FeedBackController() {}

      // Characterize and tune on the real X44 before competition use.
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final class SIM {
        private SIM() {}

        public static final double kP = 0.15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    public static final class FeedForwardController {
      private FeedForwardController() {}

      public static final double kV = 0.0;
      public static final double kS = 0.0;
      public static final double kA = 0.0;

      public static final class SIM {
        private SIM() {}

        public static final double kV = 0.02;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
      }
    }
  }

  public static final class PivotMotor {
    private PivotMotor() {}

    // Provisional CAN ID. The requested 3:1 reduction is motor rotations per arm rotation.
    public static final int MOTOR_ID = 23;
    public static final double MOTOR_GEAR_RATIO = 3.0;
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double MOTOR_INERTIA =
        PivotGeometry.MOMENT_OF_INERTIA_KG_METERS_SQUARED;
    public static final double MAX_VOLTAGE = 8.0;
    public static final double POSITION_TOLERANCE_RAD = 0.05;

    public static final class FeedBackController {
      private FeedBackController() {}

      // Deliberately left untuned. Do not enable closed-loop movement on the real robot until
      // the pivot encoder zero, direction, and gains have been verified.
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final class SIM {
        private SIM() {}

        public static final double kP = 18.0;
        public static final double kI = 0.0;
        public static final double kD = 1.2;
      }
    }

    public static final class FeedForwardController {
      private FeedForwardController() {}

      public static final double kV = 0.0;
      public static final double kS = 0.0;
      public static final double kA = 0.0;
      public static final double kG = 0.0;

      public static final class SIM {
        private SIM() {}

        // Approximate voltage required to hold the assumed 5 kg payload near horizontal.
        // Recalculate after measuring the actual arm and payload center of mass.
        public static final double kG = 6.8;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
      }
    }
  }
}
