// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

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

  /** sets coast to the climber motor. */
  public default void setCoast() {}

  /** sets brake to the climber motor. */
  public default void setBrake() {}
}
