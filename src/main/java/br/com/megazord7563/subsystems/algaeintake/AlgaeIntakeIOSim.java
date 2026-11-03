package br.com.megazord7563.subsystems.algaeintake;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation IO implementation for the AlgaeIntake subsystem. */
public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
  private final DCMotorSim pivotSim;
  private final DCMotorSim rollerSim;

  private final PIDController pivotController =
      new PIDController(
          AlgaeIntakeConstants.PivotMotor.FeedBackController.SIM.kP,
          AlgaeIntakeConstants.PivotMotor.FeedBackController.SIM.kI,
          AlgaeIntakeConstants.PivotMotor.FeedBackController.SIM.kD);
  private final PIDController rollerController =
      new PIDController(
          AlgaeIntakeConstants.RollerMotor.FeedBackController.SIM.kP,
          AlgaeIntakeConstants.RollerMotor.FeedBackController.SIM.kI,
          AlgaeIntakeConstants.RollerMotor.FeedBackController.SIM.kD);

  private boolean pivotClosedLoop;
  private boolean rollerClosedLoop;
  private double pivotAppliedVolts;
  private double rollerAppliedVolts;
  private double rollerFFVolts;

  // TODO: Add pivot gravity, mechanical travel limits, and algae interaction after defining
  // mechanism properties.
  public AlgaeIntakeIOSim() {
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                AlgaeIntakeConstants.PivotMotor.DC_MOTOR,
                AlgaeIntakeConstants.PivotMotor.MOTOR_INERTIA,
                AlgaeIntakeConstants.PivotMotor.MOTOR_GEAR_RATIO),
            AlgaeIntakeConstants.PivotMotor.DC_MOTOR);

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                AlgaeIntakeConstants.RollerMotor.DC_MOTOR,
                AlgaeIntakeConstants.RollerMotor.MOTOR_INERTIA,
                AlgaeIntakeConstants.RollerMotor.MOTOR_GEAR_RATIO),
            AlgaeIntakeConstants.RollerMotor.DC_MOTOR);
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    if (pivotClosedLoop) {
      pivotAppliedVolts = pivotController.calculate(pivotSim.getAngularPositionRad());
    } else {
      pivotController.reset();
    }

    if (rollerClosedLoop) {
      rollerAppliedVolts =
          rollerFFVolts + rollerController.calculate(rollerSim.getAngularVelocityRadPerSec());
    } else {
      rollerController.reset();
    }

    pivotAppliedVolts =
        MathUtil.clamp(
            pivotAppliedVolts,
            -AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE,
            AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE);
    rollerAppliedVolts =
        MathUtil.clamp(
            rollerAppliedVolts,
            -AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE,
            AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE);

    pivotSim.setInputVoltage(pivotAppliedVolts);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    pivotSim.update(Constants.periodicLoopPeriod);
    rollerSim.update(Constants.periodicLoopPeriod);

    inputs.pivotMotorConnected = true;
    inputs.pivotPositionRad = pivotSim.getAngularPositionRad();
    inputs.pivotVelocityRadPerSec = pivotSim.getAngularVelocityRadPerSec();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();

    inputs.rollerMotorConnected = true;
    inputs.rollerPositionRad = rollerSim.getAngularPositionRad();
    inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = rollerSim.getCurrentDrawAmps();
  }

  @Override
  public void setPivotPosition(double positionRad) {
    pivotClosedLoop = true;
    pivotController.setSetpoint(positionRad);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotClosedLoop = false;
    pivotAppliedVolts =
        MathUtil.clamp(
            volts,
            -AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE,
            AlgaeIntakeConstants.PivotMotor.MAX_VOLTAGE);
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    rollerClosedLoop = true;
    rollerFFVolts =
        AlgaeIntakeConstants.RollerMotor.FeedForwardController.SIM.kS
                * Math.signum(velocityRadPerSec)
            + AlgaeIntakeConstants.RollerMotor.FeedForwardController.SIM.kV * velocityRadPerSec;
    rollerController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerClosedLoop = false;
    rollerAppliedVolts =
        MathUtil.clamp(
            volts,
            -AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE,
            AlgaeIntakeConstants.RollerMotor.MAX_VOLTAGE);
  }

  @Override
  public void setCoast()  {
    setPivotCoast();
    setRollerCoast();
  }

  @Override 
  public void setPivotCoast() {
    pivotClosedLoop = false;
    pivotAppliedVolts = 0.0;
  }

  @Override 
  public void setRollerCoast() {
    rollerClosedLoop = false;
    rollerAppliedVolts = 0.0;
  }

  @Override
  public void setBrake()  {
    setPivotBrake();
    setRollerBrake();
  }

  @Override
  public void setPivotBrake()  {
    // In simulation, braking is the same as coasting.
    pivotClosedLoop = false;
    pivotAppliedVolts = 0.0;
  }

  @Override
  public void setRollerBrake()  {
    // In simulation, braking is the same as coasting.
    rollerClosedLoop = false;
    rollerAppliedVolts = 0.0;
  }
}
