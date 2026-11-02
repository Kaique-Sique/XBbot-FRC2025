package br.com.megazord7563.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the Climber subsystem. */
public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean motorConnected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Moves the climber to the requested mechanism position in radians. */
  public default void setPosition(double positionRad) {}

  /** Runs the climber motor with the requested open-loop voltage. */
  public default void setVoltage(double volts) {}

  /** Stops the climber motor. */
  public default void stop() {
    setVoltage(0.0);
  }
}
