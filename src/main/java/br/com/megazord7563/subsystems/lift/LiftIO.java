// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.lift;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the Lift subsystem. */
public interface LiftIO {
  @AutoLog
  public static class LiftIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderPositionRad = 0.0;
    public double leaderVelocityRadPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;

    public boolean followerMotorConnected = false;
    public double followerPositionRad = 0.0;
    public double followerVelocityRadPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LiftIOInputs inputs) {}

  /** Moves the lift to the requested mechanism position in radians using the leader. */
  public default void setPosition(double positionRad, boolean useMotionMagic) {}

  /** Runs the leader motor with the requested open-loop voltage. */
  public default void setVoltage(double volts) {}

  /** Sets both motors to coast mode through the leader and follower relationship. */
  public default void setCoast() {}

  /** Sets both motors to brake mode through the leader and follower relationship. */
  public default void setBrake() {}
}
