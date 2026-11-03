// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.outtake;

import static br.com.megazord7563.util.PhoenixUtil.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware IO implementation for the Outtake subsystem using CTRE Talon FX motor controllers.
 * Configured using parameters from {@link OuttakeConstants}.
 */
public class OuttakeIOTalonFX implements OuttakeIO {
  // Hardware objects
  private final TalonFX outtakeTalon;
  private final TalonFX pivotTalon;

  // Voltage and closed loop control requests
  private final VoltageOut outtakeVoltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage outtakeVelocityVoltageRequest = new VelocityVoltage(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);
  private final PositionVoltage pivotPositionVoltageRequest = new PositionVoltage(0.0);

  private final NeutralOut brakeRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();

  // Status signals from outtake motor
  private final StatusSignal<Angle> outtakePosition;
  private final StatusSignal<AngularVelocity> outtakeVelocity;
  private final StatusSignal<Voltage> outtakeAppliedVolts;
  private final StatusSignal<Current> outtakeCurrent;

  // Status signals from pivot motor
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;

  // Connection debouncers
  private final Debouncer outtakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public OuttakeIOTalonFX() {
    outtakeTalon = new TalonFX(OuttakeConstants.OuttakeMotor.MOTOR_ID, OuttakeConstants.CAN_BUS);
    pivotTalon = new TalonFX(OuttakeConstants.PivotMotor.MOTOR_ID, OuttakeConstants.CAN_BUS);

    // Configure outtake motor
    var outtakeConfig = new TalonFXConfiguration();
    outtakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    outtakeConfig.Voltage.PeakForwardVoltage = OuttakeConstants.OuttakeMotor.MAX_VOLTAGE;
    outtakeConfig.Voltage.PeakReverseVoltage = -OuttakeConstants.OuttakeMotor.MAX_VOLTAGE;
    outtakeConfig.Slot0.kP = OuttakeConstants.OuttakeMotor.FeedBackController.kP;
    outtakeConfig.Slot0.kI = OuttakeConstants.OuttakeMotor.FeedBackController.kI;
    outtakeConfig.Slot0.kD = OuttakeConstants.OuttakeMotor.FeedBackController.kD;
    outtakeConfig.Slot0.kV = OuttakeConstants.OuttakeMotor.FeedForwardController.kV;
    outtakeConfig.Slot0.kS = OuttakeConstants.OuttakeMotor.FeedForwardController.kS;
    outtakeConfig.Slot0.kA = OuttakeConstants.OuttakeMotor.FeedForwardController.kA;
    outtakeConfig.Feedback.SensorToMechanismRatio = OuttakeConstants.OuttakeMotor.MOTOR_GEAR_RATIO;
    outtakeConfig.CurrentLimits.StatorCurrentLimit =
        OuttakeConstants.OuttakeMotor.STATOR_CURRENT_LIMIT.in(Amps);
    outtakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    outtakeConfig.CurrentLimits.SupplyCurrentLimit =
        OuttakeConstants.OuttakeMotor.SUPPLY_CURRENT_LIMIT.in(Amps);
    outtakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    outtakeConfig.MotorOutput.Inverted =
        OuttakeConstants.OuttakeMotor.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> outtakeTalon.getConfigurator().apply(outtakeConfig, 0.25));

    // Configure pivot motor
    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Voltage.PeakForwardVoltage = OuttakeConstants.PivotMotor.MAX_VOLTAGE;
    pivotConfig.Voltage.PeakReverseVoltage = -OuttakeConstants.PivotMotor.MAX_VOLTAGE;
    pivotConfig.Slot0.kP = OuttakeConstants.PivotMotor.FeedBackController.kP;
    pivotConfig.Slot0.kI = OuttakeConstants.PivotMotor.FeedBackController.kI;
    pivotConfig.Slot0.kD = OuttakeConstants.PivotMotor.FeedBackController.kD;
    pivotConfig.Slot0.kV = OuttakeConstants.PivotMotor.FeedForwardController.kV;
    pivotConfig.Slot0.kS = OuttakeConstants.PivotMotor.FeedForwardController.kS;
    pivotConfig.Slot0.kA = OuttakeConstants.PivotMotor.FeedForwardController.kA;
    pivotConfig.Feedback.SensorToMechanismRatio = OuttakeConstants.PivotMotor.MOTOR_GEAR_RATIO;
    pivotConfig.CurrentLimits.StatorCurrentLimit =
        OuttakeConstants.PivotMotor.STATOR_CURRENT_LIMIT.in(Amps);
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit =
        OuttakeConstants.PivotMotor.SUPPLY_CURRENT_LIMIT.in(Amps);
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.MotorOutput.Inverted =
        OuttakeConstants.PivotMotor.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> pivotTalon.getConfigurator().apply(pivotConfig, 0.25));

    // Create status signals
    outtakePosition = outtakeTalon.getPosition();
    outtakeVelocity = outtakeTalon.getVelocity();
    outtakeAppliedVolts = outtakeTalon.getMotorVoltage();
    outtakeCurrent = outtakeTalon.getStatorCurrent();

    pivotPosition = pivotTalon.getPosition();
    pivotVelocity = pivotTalon.getVelocity();
    pivotAppliedVolts = pivotTalon.getMotorVoltage();
    pivotCurrent = pivotTalon.getStatorCurrent();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        outtakePosition,
        outtakeVelocity,
        outtakeAppliedVolts,
        outtakeCurrent,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent);
    ParentDevice.optimizeBusUtilizationForAll(outtakeTalon, pivotTalon);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    var outtakeStatus =
        BaseStatusSignal.refreshAll(
            outtakePosition, outtakeVelocity, outtakeAppliedVolts, outtakeCurrent);
    var pivotStatus =
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);

    // Update outtake inputs
    inputs.outtakeMotorConnected = outtakeConnectedDebounce.calculate(outtakeStatus.isOK());
    inputs.outtakeMotorPositionRad = Units.rotationsToRadians(outtakePosition.getValueAsDouble());
    inputs.outtakeMotorVelocityRadPerSec =
        Units.rotationsToRadians(outtakeVelocity.getValueAsDouble());
    inputs.outtakeMotorAppliedVolts = outtakeAppliedVolts.getValueAsDouble();
    inputs.outtakeMotorCurrentAmps = outtakeCurrent.getValueAsDouble();

    // Update pivot inputs
    inputs.pivotMotorConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
    inputs.pivotMotorPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
    inputs.pivotMotorVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
    inputs.pivotMotorAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotMotorCurrentAmps = pivotCurrent.getValueAsDouble();
  }

  @Override
  public void setOuttakeOpenLoop(double output) {
    double clampedVolts =
        Math.max(
            -OuttakeConstants.OuttakeMotor.MAX_VOLTAGE,
            Math.min(OuttakeConstants.OuttakeMotor.MAX_VOLTAGE, output));
    outtakeTalon.setControl(outtakeVoltageRequest.withOutput(clampedVolts));
  }

  @Override
  public void setPivotOpenLoop(double output) {
    double clampedVolts =
        Math.max(
            -OuttakeConstants.PivotMotor.MAX_VOLTAGE,
            Math.min(OuttakeConstants.PivotMotor.MAX_VOLTAGE, output));
    pivotTalon.setControl(pivotVoltageRequest.withOutput(clampedVolts));
  }

  @Override
  public void setOuttakeVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    outtakeTalon.setControl(outtakeVelocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setPivotPosition(double positionRad) {
    double positionRot = Units.radiansToRotations(positionRad);
    pivotTalon.setControl(pivotPositionVoltageRequest.withPosition(positionRot));
  }

  @Override
  public void setCoast() {
    setOuttakeCoast();
    setPivotCoast();
  }

  @Override 
  public void setOuttakeCoast() {
    outtakeTalon.setControl(coastRequest);
  }

  @Override 
  public void setPivotCoast() {
    pivotTalon.setControl(coastRequest);
  }

  @Override
  public void setBrake() {
    setOuttakeBrake();
    setPivotBrake();
  }

  @Override 
  public void setOuttakeBrake()
  {
    outtakeTalon.setControl(brakeRequest);
  }

  @Override 
  public void setPivotBrake()
  {
    pivotTalon.setControl(brakeRequest);
  }
}
