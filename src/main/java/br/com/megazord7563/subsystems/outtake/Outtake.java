// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.com.megazord7563.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;

import br.com.megazord7563.Constants;
import br.com.megazord7563.Constants.Mode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  private final Alert outtakeDisconnectedAlert;
  private final Alert pivotDisconnectedAlert;
  private final SysIdRoutine sysId;

  /** Creates a new Outtake subsystem. */
  public Outtake(OuttakeIO io) {
    this.io = io;

    outtakeDisconnectedAlert = new Alert("Disconnected outtake motor.", AlertType.kError);
    pivotDisconnectedAlert = new Alert("Disconnected pivot motor on outtake.", AlertType.kError);

    // Configure SysId routine for outtake characterization
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Outtake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      stop();
    }

    outtakeDisconnectedAlert.set(
        !inputs.outtakeMotorConnected && Constants.currentMode != Mode.SIM);
    pivotDisconnectedAlert.set(!inputs.pivotMotorConnected && Constants.currentMode != Mode.SIM);
  }

  // ── Setters ─────────────────────────────────────────────────────────────────

  /** Runs the outtake motor with the specified open loop voltage. */
  public void setOuttakeOpenLoop(double output) {
    io.setOuttakeOpenLoop(output);
  }

  /** Runs the pivot motor with the specified open loop voltage. */
  public void setPivotOpenLoop(double output) {
    io.setPivotOpenLoop(output);
  }

  /** Runs the outtake motor at the specified velocity in radians per second. */
  public void setOuttakeVelocity(double velocityRadPerSec) {
    io.setOuttakeVelocity(velocityRadPerSec);
  }

  /** Runs the pivot motor to the specified rotation in radians. */
  public void setPivotPosition(double positionRad) {
    io.setPivotPosition(positionRad);
  }

  /** Runs the pivot motor to the specified rotation. */
  public void setPivotPosition(Rotation2d rotation) {
    setPivotPosition(rotation.getRadians());
  }

  /** Runs the outtake with the specified output while controlling pivot to zero radians. */
  public void runCharacterization(double output) {
    io.setOuttakeOpenLoop(output);
    io.setPivotPosition(0.0);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.stop();
  }

  // ── Getters ─────────────────────────────────────────────────────────────────

  @AutoLogOutput(key = "Outtake/OuttakeVelocityRadPerSec")
  public double getOuttakeVelocityRadPerSec() {
    return inputs.outtakeMotorVelocityRadPerSec;
  }

  @AutoLogOutput(key = "Outtake/OuttakePositionRad")
  public double getOuttakePositionRad() {
    return inputs.outtakeMotorPositionRad;
  }

  @AutoLogOutput(key = "Outtake/OuttakeAppliedVolts")
  public double getOuttakeAppliedVolts() {
    return inputs.outtakeMotorAppliedVolts;
  }

  @AutoLogOutput(key = "Outtake/OuttakeCurrentAmps")
  public double getOuttakeCurrentAmps() {
    return inputs.outtakeMotorCurrentAmps;
  }

  public boolean isOuttakeConnected() {
    return inputs.outtakeMotorConnected;
  }

  @AutoLogOutput(key = "Outtake/PivotPositionRad")
  public double getPivotPositionRad() {
    return inputs.pivotMotorPositionRad;
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(inputs.pivotMotorPositionRad);
  }

  @AutoLogOutput(key = "Outtake/PivotVelocityRadPerSec")
  public double getPivotVelocityRadPerSec() {
    return inputs.pivotMotorVelocityRadPerSec;
  }

  @AutoLogOutput(key = "Outtake/PivotAppliedVolts")
  public double getPivotAppliedVolts() {
    return inputs.pivotMotorAppliedVolts;
  }

  @AutoLogOutput(key = "Outtake/PivotCurrentAmps")
  public double getPivotCurrentAmps() {
    return inputs.pivotMotorCurrentAmps;
  }

  public boolean isPivotConnected() {
    return inputs.pivotMotorConnected;
  }

  // ── Commands ────────────────────────────────────────────────────────────────

  /** Returns a command to run a quasistatic SysId test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic SysId test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Runs the outtake open loop voltage and stops when interrupted. */
  public Command runOuttakeVoltageCommand(double volts) {
    return runEnd(() -> setOuttakeOpenLoop(volts), () -> setOuttakeOpenLoop(0.0));
  }

  /** Runs the outtake at target velocity and stops when interrupted. */
  public Command runOuttakeVelocityCommand(double velocityRadPerSec) {
    return runEnd(() -> setOuttakeVelocity(velocityRadPerSec), () -> setOuttakeOpenLoop(0.0));
  }

  /** Holds the pivot at the target position in radians. */
  public Command runPivotPositionCommand(double positionRad) {
    return run(() -> setPivotPosition(positionRad));
  }

  /** Holds the pivot at the target rotation. */
  public Command runPivotPositionCommand(Rotation2d rotation) {
    return run(() -> setPivotPosition(rotation));
  }

  /** Stops all motors on the outtake. */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
