// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.outtake;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Physics simulation for the Outtake.
 *
 * <p>The pivot is modeled as a gravity-loaded single-jointed arm with mechanical travel limits.
 * The roller is modeled independently as a motor-driven rotating inertia. The Kraken X44 roller
 * uses the closest available WPILib motor model (Kraken X60 FOC) as an approximation.
 */
public class OuttakeIOSim implements OuttakeIO {
  private final SingleJointedArmSim pivotSim;
  private final DCMotorSim outtakeSim;

  private final PIDController pivotController =
      new PIDController(
          OuttakeConstants.PivotMotor.FeedBackController.SIM.kP,
          OuttakeConstants.PivotMotor.FeedBackController.SIM.kI,
          OuttakeConstants.PivotMotor.FeedBackController.SIM.kD);

  private final PIDController outtakeController =
      new PIDController(
          OuttakeConstants.OuttakeMotor.FeedBackController.SIM.kP,
          OuttakeConstants.OuttakeMotor.FeedBackController.SIM.kI,
          OuttakeConstants.OuttakeMotor.FeedBackController.SIM.kD);

  private final ArmFeedforward pivotFeedforward =
      new ArmFeedforward(
          OuttakeConstants.PivotMotor.FeedForwardController.SIM.kS,
          OuttakeConstants.PivotMotor.FeedForwardController.SIM.kG,
          OuttakeConstants.PivotMotor.FeedForwardController.SIM.kV,
          OuttakeConstants.PivotMotor.FeedForwardController.SIM.kA);

  private boolean pivotClosedLoop;
  private boolean outtakeClosedLoop;
  private double pivotGoalRad;
  private double outtakeGoalVelocityRadPerSec;
  private double pivotAppliedVolts;
  private double outtakeAppliedVolts;
  private double outtakeFeedforwardVolts;

  public OuttakeIOSim() {
    pivotSim =
        new SingleJointedArmSim(
            OuttakeConstants.PivotMotor.DC_MOTOR,
            OuttakeConstants.PivotMotor.MOTOR_GEAR_RATIO,
            OuttakeConstants.PivotGeometry.MOMENT_OF_INERTIA_KG_METERS_SQUARED,
            OuttakeConstants.PivotGeometry.ARM_LENGTH_METERS,
            Rotation2d.fromRadians(OuttakeConstants.PivotGeometry.MIN_ANGLE_RAD),
            Rotation2d.fromRadians(OuttakeConstants.PivotGeometry.MAX_ANGLE_RAD),
            true,
            Rotation2d.fromRadians(OuttakeConstants.PivotGeometry.STARTING_ANGLE_RAD));

    outtakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                OuttakeConstants.OuttakeMotor.DC_MOTOR,
                OuttakeConstants.OuttakeMotor.MOTOR_INERTIA,
                OuttakeConstants.OuttakeMotor.MOTOR_GEAR_RATIO),
            OuttakeConstants.OuttakeMotor.DC_MOTOR);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    if (pivotClosedLoop) {
      double angleRad = pivotSim.getAngleRads();
      double velocityRadPerSec = pivotSim.getVelocityRadPerSec();
      pivotAppliedVolts =
          pivotController.calculate(angleRad, pivotGoalRad)
              + pivotFeedforward.calculate(angleRad, velocityRadPerSec);
    } else {
      pivotController.reset();
    }

    if (outtakeClosedLoop) {
      outtakeAppliedVolts =
          outtakeFeedforwardVolts
              + outtakeController.calculate(
                  outtakeSim.getAngularVelocityRadPerSec(), outtakeGoalVelocityRadPerSec);
    } else {
      outtakeController.reset();
    }

    pivotAppliedVolts =
        MathUtil.clamp(
            pivotAppliedVolts,
            -OuttakeConstants.PivotMotor.MAX_VOLTAGE,
            OuttakeConstants.PivotMotor.MAX_VOLTAGE);
    outtakeAppliedVolts =
        MathUtil.clamp(
            outtakeAppliedVolts,
            -OuttakeConstants.OuttakeMotor.MAX_VOLTAGE,
            OuttakeConstants.OuttakeMotor.MAX_VOLTAGE);

    pivotSim.setInputVoltage(pivotAppliedVolts);
    outtakeSim.setInputVoltage(outtakeAppliedVolts);
    pivotSim.update(Constants.periodicLoopPeriod);
    outtakeSim.update(Constants.periodicLoopPeriod);

    inputs.pivotMotorConnected = true;
    inputs.pivotMotorPositionRad = pivotSim.getAngleRads();
    inputs.pivotMotorVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
    inputs.pivotMotorAppliedVolts = pivotAppliedVolts;
    inputs.pivotMotorCurrentAmps = pivotSim.getCurrentDrawAmps();

    inputs.outtakeMotorConnected = true;
    inputs.outtakeMotorPositionRad = outtakeSim.getAngularPositionRad();
    inputs.outtakeMotorVelocityRadPerSec = outtakeSim.getAngularVelocityRadPerSec();
    inputs.outtakeMotorAppliedVolts = outtakeAppliedVolts;
    inputs.outtakeMotorCurrentAmps = outtakeSim.getCurrentDrawAmps();
  }

  @Override
  public void setPivotPosition(double positionRad) {
    pivotClosedLoop = true;
    pivotGoalRad =
        MathUtil.clamp(
            positionRad,
            OuttakeConstants.PivotGeometry.MIN_ANGLE_RAD,
            OuttakeConstants.PivotGeometry.MAX_ANGLE_RAD);
  }

  @Override
  public void setPivotOpenLoop(double output) {
    pivotClosedLoop = false;
    pivotController.reset();
    pivotAppliedVolts =
        MathUtil.clamp(
            output,
            -OuttakeConstants.PivotMotor.MAX_VOLTAGE,
            OuttakeConstants.PivotMotor.MAX_VOLTAGE);
  }

  @Override
  public void setOuttakeVelocity(double velocityRadPerSec) {
    outtakeClosedLoop = true;
    outtakeGoalVelocityRadPerSec = velocityRadPerSec;
    outtakeFeedforwardVolts =
        OuttakeConstants.OuttakeMotor.FeedForwardController.SIM.kS
                * Math.signum(velocityRadPerSec)
            + OuttakeConstants.OuttakeMotor.FeedForwardController.SIM.kV * velocityRadPerSec;
  }

  @Override
  public void setOuttakeOpenLoop(double output) {
    outtakeClosedLoop = false;
    outtakeController.reset();
    outtakeAppliedVolts =
        MathUtil.clamp(
            output,
            -OuttakeConstants.OuttakeMotor.MAX_VOLTAGE,
            OuttakeConstants.OuttakeMotor.MAX_VOLTAGE);
  }

  @Override
  public void setCoast() {
    setPivotCoast();
    setOuttakeCoast();
  }

  @Override
  public void setPivotCoast() {
    pivotClosedLoop = false;
    pivotController.reset();
    pivotAppliedVolts = 0.0;
  }

  @Override
  public void setOuttakeCoast() {
    outtakeClosedLoop = false;
    outtakeController.reset();
    outtakeAppliedVolts = 0.0;
  }

  @Override
  public void setBrake() {
    // WPILib's simple plant model approximates brake mode as zero applied voltage.
    setPivotBrake();
    setOuttakeBrake();
  }

  @Override
  public void setPivotBrake() {
    setPivotCoast();
  }

  @Override
  public void setOuttakeBrake() {
    setOuttakeCoast();
  }
}
