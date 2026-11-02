package br.com.megazord7563.subsystems.climber;

import br.com.megazord7563.Constants;
import br.com.megazord7563.Constants.Mode;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Subsystem responsible for climbing the robot. */
public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert;

  /** Creates a new Climber subsystem. */
  public Climber(ClimberIO io) {
    this.io = io;
    motorDisconnectedAlert = new Alert("Climber motor is disconnected.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }

    motorDisconnectedAlert.set(!inputs.motorConnected && Constants.currentMode != Mode.SIM);
  }

  // ── Setters ─────────────────────────────────────────────────────────────────

  /** Moves the climber to the requested mechanism position in radians. */
  public void setPosition(double positionRad) {
    io.setPosition(positionRad);
  }

  /** Runs the climber at the requested angular velocity in radians per second. */
  public void setVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
  }

  /** Runs the climber motor with the requested open-loop voltage. */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the climber motor. */
  public void stop() {
    io.stop();
  }

  // ── Getters ─────────────────────────────────────────────────────────────────

  @AutoLogOutput(key = "Climber/PositionRad")
  public double getPositionRad() {
    return inputs.positionRad;
  }

  @AutoLogOutput(key = "Climber/VelocityRadPerSec")
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput(key = "Climber/AppliedVolts")
  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  @AutoLogOutput(key = "Climber/CurrentAmps")
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  public boolean isConnected() {
    return inputs.motorConnected;
  }

  // ── Commands ────────────────────────────────────────────────────────────────

  /** Holds the climber at the requested position until interrupted. */
  public Command setPositionCommand(double positionRad) {
    return run(() -> setPosition(positionRad));
  }

  /** Runs the climber at the requested velocity until interrupted. */
  public Command setVelocityCommand(double velocityRadPerSec) {
    return runEnd(() -> setVelocity(velocityRadPerSec), this::stop);
  }

  /** Runs the climber at the requested voltage until interrupted. */
  public Command runVoltageCommand(double volts) {
    return runEnd(() -> setVoltage(volts), this::stop);
  }

  /** Stops the climber. */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
