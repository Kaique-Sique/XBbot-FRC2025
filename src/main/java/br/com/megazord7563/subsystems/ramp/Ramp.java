// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.ramp;

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

public class Ramp extends SubsystemBase {
  private final RampIO io;
  private final RampIOInputsAutoLogged inputs = new RampIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert;
  private double goalPositionRad;
  private boolean hasActiveGoal;

  /** Creates a new Ramp subsystem. */
  public Ramp(RampIO io) {
    this.io = io;
    io.setVoltage(0.0);
    motorDisconnectedAlert = new Alert("Disconnected ramp motor.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ramp", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }

    motorDisconnectedAlert.set(!inputs.motorConnected && Constants.currentMode != Mode.SIM);
  }

  // ── Setters ─────────────────────────────────────────────────────────────────

  /** Runs the ramp motor with the specified open-loop voltage. */
  public void setVoltage(double volts) {
    hasActiveGoal = false;
    io.setVoltage(volts);
  }

  /** Moves the ramp to the specified position in radians. */
  public void setPosition(double positionRad) {
    goalPositionRad = positionRad;
    hasActiveGoal = true;
    io.setPosition(positionRad);
  }

  /** Moves the ramp to the specified rotation. */
  public void setPosition(Rotation2d rotation) {
    setPosition(rotation.getRadians());
  }

  /** Disables the ramp motor output and clears the active position goal. */
  public void stop() {
    io.setCoast();
  }

  public void setCoast() 
  {
    hasActiveGoal = false;
    io.setCoast();
  }

  public void setBrake() 
  {
    hasActiveGoal = false;
    io.setBrake();
  }


  // ── Getters ─────────────────────────────────────────────────────────────────

  /**
   * Returns whether the ramp is within the configured tolerance of its active position goal.
   *
   * <p>This reports position error only; it does not guarantee that the mechanism has settled.
   */
  @AutoLogOutput(key = "Ramp/AtGoal")
  public boolean atGoal() {
    return hasActiveGoal
        && Math.abs(getPositionRad() - goalPositionRad) <= RampConstants.POSITION_TOLERANCE_RAD;
  }

  @AutoLogOutput(key = "Ramp/PositionRad")
  public double getPositionRad() {
    return inputs.motorPositionRad;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.motorPositionRad);
  }

  @AutoLogOutput(key = "Ramp/VelocityRadPerSec")
  public double getVelocityRadPerSec() {
    return inputs.motorVelocityRadPerSec;
  }

  @AutoLogOutput(key = "Ramp/AppliedVolts")
  public double getAppliedVolts() {
    return inputs.motorAppliedVolts;
  }

  @AutoLogOutput(key = "Ramp/CurrentAmps")
  public double getCurrentAmps() {
    return inputs.motorCurrentAmps;
  }

  public boolean isConnected() {
    return inputs.motorConnected;
  }

  // ── Commands ────────────────────────────────────────────────────────────────

  /** Holds the ramp at the target position in radians. */
  public Command setPositionCommand(double positionRad) {
    return run(() -> setPosition(positionRad));
  }

  /** Holds the ramp at the target rotation. */
  public Command setPositionCommand(Rotation2d rotation) {
    return run(() -> setPosition(rotation));
  }

  /** Runs the ramp motor at the requested voltage and stops when interrupted. */
  public Command runVoltageCommand(double volts) {
    return runEnd(() -> setVoltage(volts), this::stop);
  }

  /** Stops the ramp motor. */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
