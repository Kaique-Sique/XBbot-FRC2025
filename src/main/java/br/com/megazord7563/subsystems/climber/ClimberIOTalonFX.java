package br.com.megazord7563.subsystems.climber;

import static br.com.megazord7563.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

/** Hardware IO implementation for the Climber subsystem using one CTRE Talon FX. */
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX motor;

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withSlot(1);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ClimberIOTalonFX() {
    motor = new TalonFX(ClimberConstants.ClimberMotor.MOTOR_ID, ClimberConstants.CAN_BUS);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        ClimberConstants.ClimberMotor.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.Voltage.PeakForwardVoltage = ClimberConstants.ClimberMotor.MAX_VOLTAGE;
    config.Voltage.PeakReverseVoltage = -ClimberConstants.ClimberMotor.MAX_VOLTAGE;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        ClimberConstants.ClimberMotor.CLOSED_LOOP_RAMP_PERIOD;
    config.Feedback.SensorToMechanismRatio = ClimberConstants.ClimberMotor.MOTOR_GEAR_RATIO;

    config.Slot0.kS = ClimberConstants.ClimberMotor.FeedForwardController.kS;
    config.Slot0.kV = ClimberConstants.ClimberMotor.FeedForwardController.kV;
    config.Slot0.kA = ClimberConstants.ClimberMotor.FeedForwardController.kA;
    config.Slot0.kP = ClimberConstants.ClimberMotor.FeedBackController.kP;
    config.Slot0.kI = ClimberConstants.ClimberMotor.FeedBackController.kI;
    config.Slot0.kD = ClimberConstants.ClimberMotor.FeedBackController.kD;

    config.MotionMagic.MotionMagicCruiseVelocity =
        ClimberConstants.ClimberMotor.MotionMagic.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration =
        ClimberConstants.ClimberMotor.MotionMagic.ACCELERATION;
    config.MotionMagic.MotionMagicJerk = ClimberConstants.ClimberMotor.MotionMagic.JERK;

    config.CurrentLimits.SupplyCurrentLimit =
        ClimberConstants.ClimberMotor.SUPPLY_CURRENT_LIMIT.in(edu.wpi.first.units.Units.Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit =
        ClimberConstants.ClimberMotor.STATOR_CURRENT_LIMIT.in(edu.wpi.first.units.Units.Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.motorConnected = connectedDebounce.calculate(status.isOK());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRad) {
    motor.setControl(positionRequest.withPosition(Units.radiansToRotations(positionRad)));
  }

  @Override
  public void setVoltage(double volts) {
    double clampedVolts =
        Math.max(
            -ClimberConstants.ClimberMotor.MAX_VOLTAGE,
            Math.min(ClimberConstants.ClimberMotor.MAX_VOLTAGE, volts));
    motor.setControl(voltageRequest.withOutput(clampedVolts));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }
}
