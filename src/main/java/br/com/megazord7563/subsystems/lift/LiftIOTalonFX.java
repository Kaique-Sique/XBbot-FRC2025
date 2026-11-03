// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.lift;

import static br.com.megazord7563.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Hardware IO implementation for the Lift subsystem using two CTRE Talon FX controllers. */
public class LiftIOTalonFX implements LiftIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final NeutralOut brakeRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;

  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final boolean hardwareConfigured;

  public LiftIOTalonFX() {
    boolean validIds =
        LiftConstants.LEADER_MOTOR_ID >= 0
            && LiftConstants.FOLLOWER_MOTOR_ID >= 0
            && LiftConstants.LEADER_MOTOR_ID != LiftConstants.FOLLOWER_MOTOR_ID;

    if (!validIds) {
      // Keep the robot bootable while the hardware IDs are still being confirmed.
      leaderMotor = null;
      followerMotor = null;
      leaderPosition = null;
      leaderVelocity = null;
      leaderAppliedVolts = null;
      leaderCurrent = null;
      followerPosition = null;
      followerVelocity = null;
      followerAppliedVolts = null;
      followerCurrent = null;
      hardwareConfigured = false;
      return;
    }

    leaderMotor = new TalonFX(LiftConstants.LEADER_MOTOR_ID, LiftConstants.CAN_BUS);
    followerMotor = new TalonFX(LiftConstants.FOLLOWER_MOTOR_ID, LiftConstants.CAN_BUS);

    var leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderConfig.Voltage.PeakForwardVoltage = LiftConstants.MAX_VOLTAGE;
    leaderConfig.Voltage.PeakReverseVoltage = -LiftConstants.MAX_VOLTAGE;
    leaderConfig.Feedback.SensorToMechanismRatio = LiftConstants.MOTOR_GEAR_RATIO;
    leaderConfig.Slot0.kP = LiftConstants.FeedBackController.kP;
    leaderConfig.Slot0.kI = LiftConstants.FeedBackController.kI;
    leaderConfig.Slot0.kD = LiftConstants.FeedBackController.kD;
    leaderConfig.Slot0.kS = LiftConstants.FeedForwardController.kS;
    leaderConfig.Slot0.kV = LiftConstants.FeedForwardController.kV;
    leaderConfig.Slot0.kA = LiftConstants.FeedForwardController.kA;
    leaderConfig.Slot0.kG = LiftConstants.FeedForwardController.kG;
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(leaderConfig, 0.25));

    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.Voltage.PeakForwardVoltage = LiftConstants.MAX_VOLTAGE;
    followerConfig.Voltage.PeakReverseVoltage = -LiftConstants.MAX_VOLTAGE;
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(followerConfig, 0.25));

    leaderPosition = leaderMotor.getPosition();
    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderCurrent = leaderMotor.getStatorCurrent();

    followerPosition = followerMotor.getPosition();
    followerVelocity = followerMotor.getVelocity();
    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerCurrent = followerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor);

    hardwareConfigured = true;
    enforceFollowerMode();
  }

  /** Reasserts the follower relationship; the follower has no independent control path. */
  private void enforceFollowerMode() {
    if (hardwareConfigured) {
      followerMotor.setControl(
          new Follower(
              LiftConstants.LEADER_MOTOR_ID,
              LiftConstants.FOLLOWER_OPPOSES_LEADER
                  ? MotorAlignmentValue.Opposed
                  : MotorAlignmentValue.Aligned));
    }
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    if (!hardwareConfigured) {
      inputs.leaderMotorConnected = false;
      inputs.followerMotorConnected = false;
      return;
    }

    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition, followerVelocity, followerAppliedVolts, followerCurrent);

    inputs.leaderMotorConnected = leaderConnectedDebounce.calculate(leaderStatus.isOK());
    inputs.leaderPositionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble());
    inputs.leaderVelocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();

    inputs.followerMotorConnected = followerConnectedDebounce.calculate(followerStatus.isOK());
    inputs.followerPositionRad = Units.rotationsToRadians(followerPosition.getValueAsDouble());
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble());
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();

    enforceFollowerMode();
  }

  @Override
  public void setPosition(double positionRad) {
    if (hardwareConfigured) {
      leaderMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(positionRad)));
      enforceFollowerMode();
    }
  }

  @Override
  public void setVoltage(double volts) {
    if (hardwareConfigured) {
      double clampedVolts =
          Math.max(-LiftConstants.MAX_VOLTAGE, Math.min(LiftConstants.MAX_VOLTAGE, volts));
      leaderMotor.setControl(voltageRequest.withOutput(clampedVolts));
      enforceFollowerMode();
    }
  }

  @Override
  public void setCoast() {
    if (hardwareConfigured) {
      leaderMotor.setControl(coastRequest);
      enforceFollowerMode();
    }
  }

  @Override
  public void setBrake() {
    if (hardwareConfigured) {
      leaderMotor.setControl(brakeRequest);
      enforceFollowerMode();
    }
  }
}
