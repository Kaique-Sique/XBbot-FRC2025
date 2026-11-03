package br.com.megazord7563.subsystems.lift;

import br.com.megazord7563.Constants;
import br.com.megazord7563.Constants.Mode;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Subsystem responsible for the robot elevator. */
public class Lift extends SubsystemBase {
  private final LiftIO io;
  private final LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  private final Alert leaderDisconnectedAlert;
  private final Alert followerDisconnectedAlert;
  private double goalPositionRad;
  private boolean hasActiveGoal;

  /** Creates a new Lift subsystem. */
  public Lift(LiftIO io) {
    this.io = io;
    leaderDisconnectedAlert =
        new Alert("Lift leader motor is disconnected or not configured.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert("Lift follower motor is disconnected or not configured.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Lift", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }

    leaderDisconnectedAlert.set(
        !inputs.leaderMotorConnected && Constants.currentMode != Mode.SIM);
    followerDisconnectedAlert.set(
        !inputs.followerMotorConnected && Constants.currentMode != Mode.SIM);
  }

  // ── Setters ─────────────────────────────────────────────────────────────────

  /** Moves the lift to the requested mechanism position in radians using the leader motor. */
  public void setPosition(double positionRad) {
    goalPositionRad = positionRad;
    hasActiveGoal = true;
    io.setPosition(positionRad);
  }

  /** Runs the leader motor with the requested open-loop voltage. */
  public void setVoltage(double volts) {
    hasActiveGoal = false;
    io.setVoltage(volts);
  }

  /** Stops the lift and clears the active position goal. */
  public void stop() {
    setCoast();
  }

  /** Disengages the leader output and allows the coupled motors to coast. */
  public void setCoast() {
    hasActiveGoal = false;
    io.setCoast();
  }

  /** Engages brake mode through the leader motor. */
  public void setBrake() {
    hasActiveGoal = false;
    io.setBrake();
  }

  // ── Getters ─────────────────────────────────────────────────────────────────

  /** Returns whether the lift is within tolerance of its active position goal. */
  @AutoLogOutput(key = "Lift/AtGoal")
  public boolean atGoal() {
    return hasActiveGoal
        && Math.abs(goalPositionRad - getPositionRad()) <= LiftConstants.POSITION_TOLERANCE_RAD;
  }

  @AutoLogOutput(key = "Lift/LeaderPositionRad")
  public double getPositionRad() {
    return inputs.leaderPositionRad;
  }

  @AutoLogOutput(key = "Lift/LeaderVelocityRadPerSec")
  public double getVelocityRadPerSec() {
    return inputs.leaderVelocityRadPerSec;
  }

  @AutoLogOutput(key = "Lift/LeaderAppliedVolts")
  public double getAppliedVolts() {
    return inputs.leaderAppliedVolts;
  }

  @AutoLogOutput(key = "Lift/LeaderCurrentAmps")
  public double getCurrentAmps() {
    return inputs.leaderCurrentAmps;
  }

  public boolean isLeaderConnected() {
    return inputs.leaderMotorConnected;
  }

  public boolean isFollowerConnected() {
    return inputs.followerMotorConnected;
  }

  // ── Commands ────────────────────────────────────────────────────────────────

  /** Holds the lift at the requested position until interrupted. */
  public Command setPositionCommand(double positionRad) {
    return run(() -> setPosition(positionRad));
  }

  /** Runs the lift at the requested voltage and stops when interrupted. */
  public Command runVoltageCommand(double volts) {
    return runEnd(() -> setVoltage(volts), this::stop);
  }

  /** Stops the lift. */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
