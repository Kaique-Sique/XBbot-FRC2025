package br.com.megazord7563.subsystems.ramp;

import static br.com.megazord7563.util.PhoenixUtil.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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

/** Hardware IO implementation for the Ramp subsystem using a CTRE Talon FX. */
public class RampIOTalonFX implements RampIO {
  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);

  private final NeutralOut brakeRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public RampIOTalonFX() {
    motor = new TalonFX(RampConstants.Motor.MOTOR_ID, RampConstants.CAN_BUS);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Voltage.PeakForwardVoltage = RampConstants.Motor.MAX_VOLTAGE;
    config.Voltage.PeakReverseVoltage = -RampConstants.Motor.MAX_VOLTAGE;
    config.Slot0.kP = RampConstants.Motor.FeedBackController.kP;
    config.Slot0.kI = RampConstants.Motor.FeedBackController.kI;
    config.Slot0.kD = RampConstants.Motor.FeedBackController.kD;
    config.Slot0.kV = RampConstants.Motor.FeedForwardController.kV;
    config.Slot0.kS = RampConstants.Motor.FeedForwardController.kS;
    config.Slot0.kA = RampConstants.Motor.FeedForwardController.kA;
    config.Slot0.kG = RampConstants.Motor.FeedForwardController.kG;
    config.Feedback.SensorToMechanismRatio = RampConstants.Motor.MOTOR_GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = RampConstants.Motor.STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = RampConstants.Motor.SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        RampConstants.Motor.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(RampIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.motorConnected = connectedDebounce.calculate(status.isOK());
    inputs.motorPositionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.motorVelocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.motorAppliedVolts = appliedVolts.getValueAsDouble();
    inputs.motorCurrentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    double clampedVolts =
        Math.max(
            -RampConstants.Motor.MAX_VOLTAGE, Math.min(RampConstants.Motor.MAX_VOLTAGE, volts));
    motor.setControl(voltageRequest.withOutput(clampedVolts));
  }

  @Override
  public void setPosition(double positionRad) {
    double positionRotations = Units.radiansToRotations(positionRad);
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setCoast() {
    motor.setControl(coastRequest);
  }

  @Override
  public void setBrake() {
    motor.setControl(brakeRequest);
  }
}
