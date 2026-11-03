package br.com.megazord7563.subsystems.lift;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation IO implementation for the Lift subsystem. */
public class LiftIOSim implements LiftIO {
  private final DCMotorSim liftSim;
  private final PIDController positionController =
      new PIDController(
          LiftConstants.FeedBackController.SIM.kP,
          LiftConstants.FeedBackController.SIM.kI,
          LiftConstants.FeedBackController.SIM.kD);

  private boolean closedLoop;
  private double goalPositionRad;
  private double appliedVolts;

  // TODO: Model elevator mass, drum/pulley geometry, gravity, carriage travel, and hard stops.
  public LiftIOSim() {
    liftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                LiftConstants.DC_MOTOR,
                LiftConstants.MOTOR_INERTIA,
                LiftConstants.MOTOR_GEAR_RATIO),
            LiftConstants.DC_MOTOR);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = positionController.calculate(liftSim.getAngularPositionRad(), goalPositionRad);
    }

    appliedVolts =
        MathUtil.clamp(appliedVolts, -LiftConstants.MAX_VOLTAGE, LiftConstants.MAX_VOLTAGE);
    liftSim.setInputVoltage(appliedVolts);
    liftSim.update(Constants.periodicLoopPeriod);

    inputs.leaderMotorConnected = true;
    inputs.leaderPositionRad = liftSim.getAngularPositionRad();
    inputs.leaderVelocityRadPerSec = liftSim.getAngularVelocityRadPerSec();
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = liftSim.getCurrentDrawAmps();

    // The follower is mechanically coupled to the leader and is represented by the same plant.
    inputs.followerMotorConnected = true;
    inputs.followerPositionRad = liftSim.getAngularPositionRad();
    inputs.followerVelocityRadPerSec = liftSim.getAngularVelocityRadPerSec();
    inputs.followerAppliedVolts = appliedVolts;
    inputs.followerCurrentAmps = liftSim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double positionRad) {
    closedLoop = true;
    goalPositionRad = positionRad;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    positionController.reset();
    appliedVolts = MathUtil.clamp(volts, -LiftConstants.MAX_VOLTAGE, LiftConstants.MAX_VOLTAGE);
  }

  @Override
  public void setCoast() {
    closedLoop = false;
    positionController.reset();
    appliedVolts = 0.0;
  }

  @Override
  public void setBrake() {
    // Braking is approximated as zero applied voltage in this simulation model.
    setCoast();
  }
}
