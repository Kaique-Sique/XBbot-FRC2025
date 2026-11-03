// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.algaeintake;

import br.com.megazord7563.Constants;
import br.com.megazord7563.Constants.Mode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
  private final Alert pivotDisconnectedAlert;
  private final Alert rollerDisconnectedAlert;

  private double pivotGoalPositionRad;
  private boolean hasActivePivotGoal;

  /** Creates a new AlgaeIntake subsystem. */
  public AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;
    io.setPivotVoltage(0.0);
    io.setRollerVoltage(0.0);
    pivotDisconnectedAlert = new Alert("Disconnected algae intake pivot motor.", AlertType.kError);
    rollerDisconnectedAlert =
        new Alert("Disconnected algae intake roller motor.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }

    pivotDisconnectedAlert.set(!inputs.pivotMotorConnected && Constants.currentMode != Mode.SIM);
    rollerDisconnectedAlert.set(!inputs.rollerMotorConnected && Constants.currentMode != Mode.SIM);
  }

  // ── Setters ─────────────────────────────────────────────────────────────────

  /** Moves the articulated intake to the specified position in radians. */
  public void setPivotPosition(double positionRad) {
    pivotGoalPositionRad = positionRad;
    hasActivePivotGoal = true;
    io.setPivotPosition(positionRad);
  }

  /** Moves the articulated intake to the specified rotation. */
  public void setPivotPosition(Rotation2d rotation) {
    setPivotPosition(rotation.getRadians());
  }

  /** Runs the pivot motor with the specified open-loop voltage. */
  public void setPivotVoltage(double volts) {
    hasActivePivotGoal = false;
    io.setPivotVoltage(volts);
  }

  /** Runs the collection wheels at the specified angular velocity in radians per second. */
  public void setRollerVelocity(double velocityRadPerSec) {
    io.setRollerVelocity(velocityRadPerSec);
  }

  /** Runs the collection wheels with the specified open-loop voltage. */
  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  /** Disables outputs to both motors and clears the active pivot goal. */
  public void stop() {
    setCoast();
  }

  /** Disables all outputs to motors. */
  public void setCoast() {
    hasActivePivotGoal = false;
    io.setCoast();
  }

  /** Sets all outputs to motors to brake mode. */
  public void setBrake() {
    hasActivePivotGoal = false;
    io.setBrake();
  }

  // ── Getters ─────────────────────────────────────────────────────────────────

  /**
   * Returns whether the pivot is within the configured tolerance of its active position goal.
   *
   * <p>This reports position error only; it does not guarantee that the mechanism has settled.
   */
  @AutoLogOutput(key = "AlgaeIntake/PivotAtGoal")
  public boolean atGoal() {
    return hasActivePivotGoal
        && Math.abs(getPivotPositionRad() - pivotGoalPositionRad)
            <= AlgaeIntakeConstants.PIVOT_POSITION_TOLERANCE_RAD;
  }

  @AutoLogOutput(key = "AlgaeIntake/PivotPositionRad")
  public double getPivotPositionRad() {
    return inputs.pivotPositionRad;
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(inputs.pivotPositionRad);
  }

  @AutoLogOutput(key = "AlgaeIntake/PivotVelocityRadPerSec")
  public double getPivotVelocityRadPerSec() {
    return inputs.pivotVelocityRadPerSec;
  }

  @AutoLogOutput(key = "AlgaeIntake/PivotAppliedVolts")
  public double getPivotAppliedVolts() {
    return inputs.pivotAppliedVolts;
  }

  @AutoLogOutput(key = "AlgaeIntake/PivotCurrentAmps")
  public double getPivotCurrentAmps() {
    return inputs.pivotCurrentAmps;
  }

  public boolean isPivotConnected() {
    return inputs.pivotMotorConnected;
  }

  @AutoLogOutput(key = "AlgaeIntake/RollerPositionRad")
  public double getRollerPositionRad() {
    return inputs.rollerPositionRad;
  }

  @AutoLogOutput(key = "AlgaeIntake/RollerVelocityRadPerSec")
  public double getRollerVelocityRadPerSec() {
    return inputs.rollerVelocityRadPerSec;
  }

  @AutoLogOutput(key = "AlgaeIntake/RollerAppliedVolts")
  public double getRollerAppliedVolts() {
    return inputs.rollerAppliedVolts;
  }

  @AutoLogOutput(key = "AlgaeIntake/RollerCurrentAmps")
  public double getRollerCurrentAmps() {
    return inputs.rollerCurrentAmps;
  }

  public boolean isRollerConnected() {
    return inputs.rollerMotorConnected;
  }

  // ── Commands ────────────────────────────────────────────────────────────────

  /** Holds the intake pivot at the target position in radians. */
  public Command setPivotPositionCommand(double positionRad) {
    return run(() -> setPivotPosition(positionRad));
  }

  /** Holds the intake pivot at the target rotation. */
  public Command setPivotPositionCommand(Rotation2d rotation) {
    return run(() -> setPivotPosition(rotation));
  }

  /** Runs the collection wheels at the requested velocity until interrupted. */
  public Command runRollerVelocityCommand(double velocityRadPerSec) {
    return runEnd(() -> setRollerVelocity(velocityRadPerSec), () -> setRollerVoltage(0.0));
  }

  /** Runs the collection wheels at the requested voltage until interrupted. */
  public Command runRollerVoltageCommand(double volts) {
    return runEnd(() -> setRollerVoltage(volts), () -> setRollerVoltage(0.0));
  }

  /** Runs the pivot at the requested voltage until interrupted. */
  public Command runPivotVoltageCommand(double volts) {
    return runEnd(() -> setPivotVoltage(volts), () -> setPivotVoltage(0.0));
  }

  /** Stops both motors. */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
