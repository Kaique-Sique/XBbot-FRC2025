// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.ramp;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation IO implementation for the Ramp subsystem. */
public class RampIOSim implements RampIO {
  private final DCMotorSim motorSim;
  private final PIDController controller =
      new PIDController(
          RampConstants.Motor.FeedBackController.SIM.kP,
          RampConstants.Motor.FeedBackController.SIM.kI,
          RampConstants.Motor.FeedBackController.SIM.kD);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  // TODO: Add gravity and mechanical travel limits after the ramp geometry and mass properties are
  // defined.
  public RampIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                RampConstants.Motor.DC_MOTOR,
                RampConstants.Motor.MOTOR_INERTIA,
                RampConstants.Motor.MOTOR_GEAR_RATIO),
            RampConstants.Motor.DC_MOTOR);
  }

  @Override
  public void updateInputs(RampIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = controller.calculate(motorSim.getAngularPositionRad());
    } else {
      controller.reset();
    }

    appliedVolts =
        MathUtil.clamp(
            appliedVolts, -RampConstants.Motor.MAX_VOLTAGE, RampConstants.Motor.MAX_VOLTAGE);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(Constants.periodicLoopPeriod);

    inputs.motorConnected = true;
    inputs.motorPositionRad = motorSim.getAngularPositionRad();
    inputs.motorVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.motorAppliedVolts = appliedVolts;
    inputs.motorCurrentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts =
        MathUtil.clamp(volts, -RampConstants.Motor.MAX_VOLTAGE, RampConstants.Motor.MAX_VOLTAGE);
  }

  @Override
  public void setPosition(double positionRad) {
    closedLoop = true;
    controller.setSetpoint(positionRad);
  }

  @Override
  public void setCoast() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void setBrake() {
    //Same as coast in simulation mode
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
