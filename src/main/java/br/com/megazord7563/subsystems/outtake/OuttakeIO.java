package br.com.megazord7563.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

    @AutoLog
    public class OuttakeIOInputs
    {
        public boolean outakeMotorConnected = false;
        public double outakeMotorPositionRad = 0.0;
        public double outakeMotorVelocityRadPerSec = 0.0;
        public double outakeMotorAppliedVolts = 0.0;
        public double outakeMotorCurrentAmps = 0.0;

        public boolean pivotMotorConnected = false;
        public double pivotMotorPositionRad = 0.0;
        public double pivotMotorVelocityRadPerSec = 0.0;
        public double pivotMotorAppliedVolts = 0.0;
        public double pivotMotorCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(OuttakeIOInputs inputs) {}

    /** Run the outtake motor at the specified open loop value. */
    public default void setOuttakeOpenLoop(double output) {}

    /** Run the pivot motor at the specified open loop value. */
    public default void setPivotOpenLoop(double output) {}

    /** Run the outtake motor at the specified velocity. */
    public default void setOuttakeVelocity(double velocityRadPerSec) {}

    /** Run the pivot motor to the specified rotation. */
    public default void setPivotPosition(double positionRad) {}
}