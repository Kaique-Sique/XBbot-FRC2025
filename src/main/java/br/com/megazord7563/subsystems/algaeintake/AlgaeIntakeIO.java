package br.com.megazord7563.subsystems.algaeintake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
  @AutoLog
  public static class AlgaeIntakeIOInputs {
    public boolean pivotMotorConnected = false;
    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public boolean rollerMotorConnected = false;
    public double rollerPositionRad = 0.0;
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  /** Moves the articulated intake to the specified position in radians. */
  public default void setPivotPosition(double positionRad) {}

  /** Runs the pivot motor with the specified open-loop voltage. */
  public default void setPivotVoltage(double volts) {}

  /** Runs the collection wheels at the specified angular velocity in radians per second. */
  public default void setRollerVelocity(double velocityRadPerSec) {}

  /** Runs the collection wheels with the specified open-loop voltage. */
  public default void setRollerVoltage(double volts) {}

  /** Disables outputs to both motors. */
  public default void stop() {
    setPivotVoltage(0.0);
    setRollerVoltage(0.0);
  }
}
