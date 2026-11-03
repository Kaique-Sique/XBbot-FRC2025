package br.com.megazord7563.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {
  @AutoLog
  public static class RampIOInputs {
    public boolean motorConnected = false;
    public double motorPositionRad = 0.0;
    public double motorVelocityRadPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(RampIOInputs inputs) {}

  /** Runs the ramp motor with the specified open-loop voltage. */
  public default void setVoltage(double volts) {}

  /** Moves the ramp to the specified position in radians. */
  public default void setPosition(double positionRad) {}

  /** Sets the motor to coast mode. */
  public default void setCoast() {}

  /** Sets the motor to brake mode. */
  public default void setBrake() {}
}
