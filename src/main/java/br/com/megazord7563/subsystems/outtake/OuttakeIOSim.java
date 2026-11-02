package br.com.megazord7563.subsystems.outtake;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of Outtake IO. The sim models are configured using the motor and
 * controller constants from OuttakeConstants. Simulation is voltage controlled.
 */
public class OuttakeIOSim implements OuttakeIO {
  private final DCMotorSim outtakeSim;
  private final DCMotorSim pivotSim;

  private boolean outtakeClosedLoop = false;
  private boolean pivotClosedLoop = false;

  private final PIDController outtakeController =
      new PIDController(
          OuttakeConstants.OuttakeMotor.FeedBackController.SIM.kP,
          OuttakeConstants.OuttakeMotor.FeedBackController.SIM.kI,
          OuttakeConstants.OuttakeMotor.FeedBackController.SIM.kD);

  private final PIDController pivotController =
      new PIDController(
          OuttakeConstants.PivotMotor.FeedBackController.SIM.kP,
          OuttakeConstants.PivotMotor.FeedBackController.SIM.kI,
          OuttakeConstants.PivotMotor.FeedBackController.SIM.kD);

  private double outtakeFFVolts = 0.0;
  private double pivotFFVolts = 0.0;
  private double outtakeAppliedVolts = 0.0;
  private double pivotAppliedVolts = 0.0;

  public OuttakeIOSim() {
    // Create outtake and pivot sim models
    outtakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                OuttakeConstants.OuttakeMotor.DC_MOTOR,
                OuttakeConstants.OuttakeMotor.MOTOR_INERTIA,
                OuttakeConstants.OuttakeMotor.MOTOR_GEAR_RATIO),
            OuttakeConstants.OuttakeMotor.DC_MOTOR);

    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                OuttakeConstants.PivotMotor.DC_MOTOR,
                OuttakeConstants.PivotMotor.MOTOR_INERTIA,
                OuttakeConstants.PivotMotor.MOTOR_GEAR_RATIO),
            OuttakeConstants.PivotMotor.DC_MOTOR);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    // Run closed-loop control
    if (outtakeClosedLoop) {
      outtakeAppliedVolts =
          outtakeFFVolts + outtakeController.calculate(outtakeSim.getAngularVelocityRadPerSec());
    } else {
      outtakeController.reset();
    }
    if (pivotClosedLoop) {
      pivotAppliedVolts =
          pivotFFVolts + pivotController.calculate(pivotSim.getAngularPositionRad());
    } else {
      pivotController.reset();
    }

    // Update simulation state
    outtakeSim.setInputVoltage(
        MathUtil.clamp(
            outtakeAppliedVolts,
            -OuttakeConstants.OuttakeMotor.MAX_VOLTAGE,
            OuttakeConstants.OuttakeMotor.MAX_VOLTAGE));
    pivotSim.setInputVoltage(
        MathUtil.clamp(
            pivotAppliedVolts,
            -OuttakeConstants.PivotMotor.MAX_VOLTAGE,
            OuttakeConstants.PivotMotor.MAX_VOLTAGE));
    outtakeSim.update(Constants.periodicLoopPeriod);
    pivotSim.update(Constants.periodicLoopPeriod);

    // Update outtake inputs
    inputs.outtakeMotorConnected = true;
    inputs.outtakeMotorPositionRad = outtakeSim.getAngularPositionRad();
    inputs.outtakeMotorVelocityRadPerSec = outtakeSim.getAngularVelocityRadPerSec();
    inputs.outtakeMotorAppliedVolts = outtakeAppliedVolts;
    inputs.outtakeMotorCurrentAmps = outtakeSim.getCurrentDrawAmps();

    // Update pivot inputs
    inputs.pivotMotorConnected = true;
    inputs.pivotMotorPositionRad = pivotSim.getAngularPositionRad();
    inputs.pivotMotorVelocityRadPerSec = pivotSim.getAngularVelocityRadPerSec();
    inputs.pivotMotorAppliedVolts = pivotAppliedVolts;
    inputs.pivotMotorCurrentAmps = pivotSim.getCurrentDrawAmps();
  }

  @Override
  public void setOuttakeOpenLoop(double output) {
    outtakeClosedLoop = false;
    outtakeAppliedVolts =\n        MathUtil.clamp(\n            output,\n            -OuttakeConstants.OuttakeMotor.MAX_VOLTAGE,\n            OuttakeConstants.OuttakeMotor.MAX_VOLTAGE);
  }

  @Override
  public void setPivotOpenLoop(double output) {
    pivotClosedLoop = false;
    pivotAppliedVolts =\n        MathUtil.clamp(\n            output,\n            -OuttakeConstants.PivotMotor.MAX_VOLTAGE,\n            OuttakeConstants.PivotMotor.MAX_VOLTAGE);
  }

  @Override
  public void setOuttakeVelocity(double velocityRadPerSec) {
    outtakeClosedLoop = true;
    outtakeFFVolts =
        OuttakeConstants.OuttakeMotor.FeedForwardController.SIM.kS * Math.signum(velocityRadPerSec)
            + OuttakeConstants.OuttakeMotor.FeedForwardController.SIM.kV * velocityRadPerSec;
    outtakeController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setPivotPosition(double positionRad) {
    pivotClosedLoop = true;
    pivotFFVolts =
        OuttakeConstants.PivotMotor.FeedForwardController.SIM.kS * Math.signum(positionRad)
            + OuttakeConstants.PivotMotor.FeedForwardController.SIM.kV * positionRad;
    pivotController.setSetpoint(positionRad);
  }

  @Override
  public void stop() {
    setOuttakeOpenLoop(0.0);
    setPivotOpenLoop(0.0);
  }
}
