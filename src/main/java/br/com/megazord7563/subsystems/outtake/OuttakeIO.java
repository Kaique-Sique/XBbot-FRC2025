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

  /** Disables outputs to all motors. */
  public default void stop() {
    setOuttakeOpenLoop(0.0);
    setPivotOpenLoop(0.0);
  }
}
