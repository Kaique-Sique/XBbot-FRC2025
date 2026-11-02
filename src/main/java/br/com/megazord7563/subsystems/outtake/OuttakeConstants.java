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
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  public static final class OuttakeMotor {
    public static final int MOTOR_ID = 0; // TODO: Set a unique CAN ID before enabling this motor on the robot.
    public static final double MOTOR_GEAR_RATIO = 1.0; // TODO: Replace with the measured mechanism reduction.
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

    public static final double MOTOR_INERTIA = 0.2; // TODO: Measure or calculate the mechanism-side moment of inertia.
    public static final double MAX_VOLTAGE = 8.0;

    // TODO: Characterize and tune these gains before using closed-loop control.
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

    // TODO: Add a characterized gravity feedforward (kG) after defining the pivot zero angle.
    public static class FeedForwardController {\n      public static final double kV = 0.0;\n      public static final double kS = 0.0;\n      public static final double kA = 0.0;\n\n      public static class SIM {
        public static final double kV = 0.0;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
      }
    }
  }

  public static final class PivotMotor {
    public static final int MOTOR_ID = 0; // TODO: Set a unique CAN ID before enabling this motor on the robot.
    public static final double MOTOR_GEAR_RATIO = 1.0;
    public static final boolean MOTOR_INVERTED = false;
    public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(1);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

    public static final double MOTOR_INERTIA = 0.4; // TODO: Measure or calculate the pivot-side moment of inertia.
    public static final double MAX_VOLTAGE = 8.0;

    // TODO: Calibrate Gains
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
