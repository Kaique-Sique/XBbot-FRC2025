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
          ClimberConstants.ClimberMotor.FeedBackController.SIM.kP,
          ClimberConstants.ClimberMotor.FeedBackController.SIM.kI,
          ClimberConstants.ClimberMotor.FeedBackController.SIM.kD);

  private boolean climberClosedLoop = false;
  private double climberFFVolts = 0.0;
  private double appliedVolts = 0.0;

  public ClimberIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.ClimberMotor.DC_MOTOR,
                ClimberConstants.ClimberMotor.MOTOR_INERTIA,
                ClimberConstants.ClimberMotor.MOTOR_GEAR_RATIO),
            ClimberConstants.ClimberMotor.DC_MOTOR);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if(climberClosedLoop)
    {
      appliedVolts = 
        climberFFVolts + positionController.calculate(motorSim.getAngularPositionRad());
    }
    else
    {
      positionController.reset();
    }

    appliedVolts =
        MathUtil.clamp(
            appliedVolts,
            -ClimberConstants.ClimberMotor.MAX_VOLTAGE,
            ClimberConstants.ClimberMotor.MAX_VOLTAGE);
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
    climberClosedLoop = true;
    positionController.setSetpoint(positionRad);
  }

  @Override
  public void setVoltage(double volts) {
    climberClosedLoop = false;
    appliedVolts =
        MathUtil.clamp(
            volts,
            -ClimberConstants.ClimberMotor.MAX_VOLTAGE,
            ClimberConstants.ClimberMotor.MAX_VOLTAGE);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
