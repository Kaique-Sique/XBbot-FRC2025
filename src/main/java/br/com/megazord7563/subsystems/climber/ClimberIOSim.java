package br.com.megazord7563.subsystems.climber;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation IO implementation for the Climber subsystem. */
public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim motorSim;
  private final PIDController positionController =
      new PIDController(
          ClimberConstants.Motor.SIM_POSITION_KP,
          ClimberConstants.Motor.SIM_POSITION_KI,
          ClimberConstants.Motor.SIM_POSITION_KD);
  private final PIDController velocityController =
      new PIDController(
          ClimberConstants.Motor.SIM_VELOCITY_KP,
          ClimberConstants.Motor.SIM_VELOCITY_KI,
          ClimberConstants.Motor.SIM_VELOCITY_KD);

  private enum ControlMode {
    VOLTAGE,
    POSITION,
    VELOCITY
  }

  private ControlMode controlMode = ControlMode.VOLTAGE;
  private double appliedVolts;

  public ClimberIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.Motor.DC_MOTOR,
                ClimberConstants.Motor.SIM_MOTOR_INERTIA,
                ClimberConstants.Motor.MOTOR_GEAR_RATIO),
            ClimberConstants.Motor.DC_MOTOR);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    switch (controlMode) {
      case POSITION:
        appliedVolts = positionController.calculate(motorSim.getAngularPositionRad());
        velocityController.reset();
        break;
      case VELOCITY:
        appliedVolts = velocityController.calculate(motorSim.getAngularVelocityRadPerSec());
        positionController.reset();
        break;
      default:
        positionController.reset();
        velocityController.reset();
        break;
    }

    appliedVolts =
        MathUtil.clamp(
            appliedVolts,
            -ClimberConstants.Motor.MAX_VOLTAGE,
            ClimberConstants.Motor.MAX_VOLTAGE);
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(Constants.periodicLoopPeriod);

    inputs.motorConnected = true;
    inputs.positionRad = motorSim.getAngularPositionRad();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double positionRad) {
    controlMode = ControlMode.POSITION;
    positionController.setSetpoint(positionRad);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    controlMode = ControlMode.VELOCITY;
    velocityController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setVoltage(double volts) {
    controlMode = ControlMode.VOLTAGE;
    appliedVolts =
        MathUtil.clamp(
            volts,
            -ClimberConstants.Motor.MAX_VOLTAGE,
            ClimberConstants.Motor.MAX_VOLTAGE);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
