// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.algaeintake;

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

/** Hardware IO implementation for the AlgaeIntake subsystem using CTRE Talon FX controllers. */
public class AlgaeIntakeIOTalonFX implements AlgaeIntakeIO {
  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;

  private final PositionVoltage pivotPositionRequest = new PositionVoltage(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);

  private final NeutralOut brakeRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;

  private final StatusSignal<Angle> rollerPosition;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;

  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public AlgaeIntakeIOTalonFX() {
    pivotMotor =
        new TalonFX(AlgaeIntakeConstants.PivotMotor.MOTOR_ID, AlgaeIntakeConstants.CAN_BUS);
    rollerMotor =
        new TalonFX(AlgaeIntakeConstants.RollerMotor.MOTOR_ID, AlgaeIntakeConstants.CAN_BUS);

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Voltage.PeakForwardVoltage = AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE;
    pivotConfig.Voltage.PeakReverseVoltage = -AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE;
    pivotConfig.Slot0.kP = AlgaeIntakeConstants.PivotMotor.FeedBackController.kP;
    pivotConfig.Slot0.kI = AlgaeIntakeConstants.PivotMotor.FeedBackController.kI;
    pivotConfig.Slot0.kD = AlgaeIntakeConstants.PivotMotor.FeedBackController.kD;
    pivotConfig.Slot0.kV = AlgaeIntakeConstants.PivotMotor.FeedForwardController.kV;
    pivotConfig.Slot0.kS = AlgaeIntakeConstants.PivotMotor.FeedForwardController.kS;
    pivotConfig.Slot0.kA = AlgaeIntakeConstants.PivotMotor.FeedForwardController.kA;
    pivotConfig.Slot0.kG = AlgaeIntakeConstants.PivotMotor.FeedForwardController.kG;
    pivotConfig.Feedback.SensorToMechanismRatio = AlgaeIntakeConstants.PivotMotor.MOTOR_GEAR_RATIO;
    pivotConfig.CurrentLimits.StatorCurrentLimit =
        AlgaeIntakeConstants.PivotMotor.STATOR_CURRENT_LIMIT.in(Amps);
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit =
        AlgaeIntakeConstants.PivotMotor.SUPPLY_CURRENT_LIMIT.in(Amps);
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.MotorOutput.Inverted =
        AlgaeIntakeConstants.PivotMotor.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Voltage.PeakForwardVoltage = AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE;
    rollerConfig.Voltage.PeakReverseVoltage = -AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE;
    rollerConfig.Slot0.kP = AlgaeIntakeConstants.RollerMotor.FeedBackController.kP;
    rollerConfig.Slot0.kI = AlgaeIntakeConstants.RollerMotor.FeedBackController.kI;
    rollerConfig.Slot0.kD = AlgaeIntakeConstants.RollerMotor.FeedBackController.kD;
    rollerConfig.Slot0.kV = AlgaeIntakeConstants.RollerMotor.FeedForwardController.kV;
    rollerConfig.Slot0.kS = AlgaeIntakeConstants.RollerMotor.FeedForwardController.kS;
    rollerConfig.Slot0.kA = AlgaeIntakeConstants.RollerMotor.FeedForwardController.kA;
    rollerConfig.Feedback.SensorToMechanismRatio =
        AlgaeIntakeConstants.RollerMotor.MOTOR_GEAR_RATIO;
    rollerConfig.CurrentLimits.StatorCurrentLimit =
        AlgaeIntakeConstants.RollerMotor.STATOR_CURRENT_LIMIT.in(Amps);
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        AlgaeIntakeConstants.RollerMotor.SUPPLY_CURRENT_LIMIT.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.Inverted =
        AlgaeIntakeConstants.RollerMotor.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));

    pivotPosition = pivotMotor.getPosition();
    pivotVelocity = pivotMotor.getVelocity();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getStatorCurrent();

    rollerPosition = rollerMotor.getPosition();
    rollerVelocity = rollerMotor.getVelocity();
    rollerAppliedVolts = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(pivotMotor, rollerMotor);
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    var pivotStatus =
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    var rollerStatus =
        BaseStatusSignal.refreshAll(
            rollerPosition, rollerVelocity, rollerAppliedVolts, rollerCurrent);

    inputs.pivotMotorConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
    inputs.pivotPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
    inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();

    inputs.rollerMotorConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.rollerPositionRad = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
    inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double positionRad) {
    pivotMotor.setControl(pivotPositionRequest.withPosition(Units.radiansToRotations(positionRad)));
  }

  @Override
  public void setPivotVoltage(double volts) {
    double clampedVolts =
        Math.max(
            -AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE,
            Math.min(AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE, volts));
    pivotMotor.setControl(pivotVoltageRequest.withOutput(clampedVolts));
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    rollerMotor.setControl(
        rollerVelocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setRollerVoltage(double volts) {
    double clampedVolts =
        Math.max(
            -AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE,
            Math.min(AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE, volts));
    rollerMotor.setControl(rollerVoltageRequest.withOutput(clampedVolts));
  }

  @Override
  public void setCoast() {
    setPivotCoast();
    setPivotCoast();
  }

  @Override
  public void setPivotCoast() {
    pivotMotor.setControl(coastRequest);
  }

  @Override
  public void setRollerCoast() {
    rollerMotor.setControl(coastRequest);
  }

  @Override
  public void setBrake() {
    setPivotBrake();
    setRollerBrake();
  }

  @Override
  public void setPivotBrake() {
    pivotMotor.setControl(brakeRequest);
  }

  @Override
  public void setRollerBrake() {
    rollerMotor.setControl(brakeRequest);
  }
}
