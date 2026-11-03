// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public boolean outtakeMotorConnected = false;
    public double outtakeMotorPositionRad = 0.0;
    public double outtakeMotorVelocityRadPerSec = 0.0;
    public double outtakeMotorAppliedVolts = 0.0;
    public double outtakeMotorCurrentAmps = 0.0;

    public boolean pivotMotorConnected = false;
    public double pivotMotorPositionRad = 0.0;
    public double pivotMotorVelocityRadPerSec = 0.0;
    public double pivotMotorAppliedVolts = 0.0;
    public double pivotMotorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(OuttakeIOInputs inputs) {}

  /** Run the outtake motor at the specified open loop value. */
  public default void setOuttakeOpenLoop(double output) {}

  /** Run the pivot motor at the specified open loop value. */
  public default void setPivotOpenLoop(double output) {}

  /** Run the outtake motor at the specified velocity. */
  public default void setOuttakeVelocity(double velocityRadPerSec) {}

  /** Run the pivot motor to the specified rotation. */
  public default void setPivotPosition(double positionRad) {}

  /** Sets the only outtake motor to coast mode. */
  public default void setOuttakeCoast() {}

  /** Sets the only pivot motor to coast mode. */
  public default void setPivotCoast() {}
  
  /** Sets the only outtake motor to brake mode. */
  public default void setOuttakeBrake() {}

  /** Sets the only pivot motor to brake mode. */
  public default void setPivotBrake() {}

  /** Disables outputs to all motors. */
  public default void setCoast() {
    setOuttakeCoast();
    setPivotCoast();
  }

  /** Brake all motors. */
  public default void setBrake() {
    setOuttakeBrake(); 
    setPivotBrake();
  }
}
