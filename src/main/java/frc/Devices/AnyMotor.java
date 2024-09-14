package frc.Devices;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MathPlus;
import frc.lib.util.MotionController;

/**
 * AnyMotor is an abstract base class representing a motor with basic
 * functionalities.
 * This class should be extended to implement specific types of motors.
 */
public abstract class AnyMotor extends SubsystemBase {

    double lastVoltage = 0;
    protected boolean isReversed; // Flag indicating whether the motor's direction is reversed.

    MotionController con;

    Double targetSpeed;

    double maxAbsVoltage = 12;

    public void setMaxVoltage(double voltage) {
        maxAbsVoltage = voltage;
    }

    public void setVelocityPD(MotionController con) {
        this.con = con.clone();
    }

    /**
     * Sets motor velocity in rotations/sec
     * 
     * @param vel
     */
    public void setVelocity(double vel) {
        if (con == null)
            throw new Error("Motor Controller not configured to control speed");
        targetSpeed = isReversed ? -vel : vel;
    }

    @Override
    public void periodic() {
        final double dTime = 0.02;
        if (targetSpeed != null) {
            // reversed in setTargetSpeed
            var voltage = con.solve(targetSpeed - uGetVelocity(), dTime);
            voltage = MathPlus.clampAbsVal(voltage, maxAbsVoltage);
            uSetVoltage(voltage);
            this.lastVoltage = isReversed ? -voltage : voltage;
        }
    }

    /**
     * Gets the unique identifier of the motor.
     * 
     * @return The motor's ID.
     */
    public abstract int getID();

    /**
     * Constructs an AnyMotor with specified reversal status.
     * 
     * @param isReversed Whether the motor direction is reversed.
     */
    public AnyMotor(boolean isReversed) {
        this.isReversed = isReversed;
    }

    /**
     * Sets the current limit of the motor.
     * 
     * @param amps The current limit in amperes.
     */
    public abstract void setCurrentLimit(int amps);

    /**
     * Internal method to set the voltage of the motor.
     * 
     * @param voltage The voltage to be set.
     */
    protected abstract void uSetVoltage(double voltage);

    /**
     * Internal method to get the revolutions of the motor.
     * 
     * @return The number of revolutions.
     */
    protected abstract double uGetRevs();

    double resetPos = 0; // The position of the encoder when last reset.

    /**
     * Resets the motor encoder to zero.
     */
    public void resetEncoder() {
        setEncoder(0);
    }

    public void setEncoder(double val) {
        resetPos = uGetRevs() - val;
    }

    /**
     * Gets the number of revolutions since the last reset, accounting for motor
     * direction.
     * 
     * @return The number of revolutions.
     */
    public double getRevs() {
        var pos = uGetRevs() - resetPos;
        return (isReversed ? -pos : pos);
    }

    public double getVelocity() {
        var vel = uGetVelocity();
        return (isReversed ? -vel : vel);
    }

    public double getVoltage() {
        return lastVoltage;
    }

    /**
     * Converts revolutions to degrees.
     * 
     * @return The number of degrees rotated since the last reset.
     */
    public double getDegrees() {
        return getRevs() * 360.0;
    }

    /**
     * Converts revolutions to radians.
     * 
     * @return The number of radians rotated since the last reset.
     */
    public double getRadians() {
        return getRevs() * 2.0 * Math.PI;
    }

    /**
     * Stops the motor by setting the voltage to zero.
     */
    public abstract void stop();

    /**
     * Sets the voltage applied to the motor, respecting the motor direction.
     * 
     * @param volts The voltage to be applied.
     */
    public void setVoltage(double volts) {
        targetSpeed = null;
        volts = isReversed ? -volts : volts;
        volts = MathPlus.clampAbsVal(volts, maxAbsVoltage);
        uSetVoltage(volts);
        this.lastVoltage = volts;
    }

    /**
     * Velocity in rotations/second
     */
    protected abstract double uGetVelocity();

    /**
     * Sets the voltage as a percentage of the maximum voltage.
     * 
     * @param percent The percentage of the maximum voltage to apply.
     */
    public void setPercentVoltage(double percent) {
        setVoltage(percent * (12.0 / 100.0));
    }
}