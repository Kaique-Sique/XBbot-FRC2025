// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.climber;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation IO implementation for the Climber subsystem. */
public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim motorSim;
  private final ProfiledPIDController positionController =
      new ProfiledPIDController(
          ClimberConstants.ClimberMotor.FeedBackController.SIM.kP,
          ClimberConstants.ClimberMotor.FeedBackController.SIM.kI,
          ClimberConstants.ClimberMotor.FeedBackController.SIM.kD,
          new Constraints(
              ClimberConstants.ClimberMotor.MotionMagic.CRUISE_VELOCITY * 2.0 * Math.PI,
              ClimberConstants.ClimberMotor.MotionMagic.ACCELERATION * 2.0 * Math.PI));
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          ClimberConstants.ClimberMotor.FeedForwardController.SIM.kS,
          ClimberConstants.ClimberMotor.FeedForwardController.SIM.kV,
          ClimberConstants.ClimberMotor.FeedForwardController.SIM.kA);

  private boolean closedLoop = false;
  private double goalPositionRad = 0.0;
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
    if (closedLoop) {
      double feedbackVolts =
          positionController.calculate(motorSim.getAngularPositionRad(), goalPositionRad);
      double feedforwardVolts = feedforward.calculate(positionController.getSetpoint().velocity);
      appliedVolts = feedbackVolts + feedforwardVolts;
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
    if (!closedLoop) {
      positionController.reset(
          motorSim.getAngularPositionRad(), motorSim.getAngularVelocityRadPerSec());
    }
    closedLoop = true;
    goalPositionRad = positionRad;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts =
        MathUtil.clamp(
            volts,
            -ClimberConstants.ClimberMotor.MAX_VOLTAGE,
            ClimberConstants.ClimberMotor.MAX_VOLTAGE);
  }

  @Override
  public void setCoast() 
  {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override 
  public void setBrake()
  {
    // Same as coast on simulation mode
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
