package frc.Devices.Motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.Devices.AnyMotor;

/**
 * The Falcon class extends the AnyMotor abstract class to provide an interface
 * to control a Talon FX motor controller (also known as a Falcon 500).
 */
public class TalonFX extends AnyMotor {
    private com.ctre.phoenix6.hardware.TalonFX talon; // The Talon FX motor controller object.

    final int id; // Unique identifier for the motor controller.

    /**
     * Retrieves the ID of the motor controller.
     * 
     * @return The CAN ID of the motor controller.
     */
    public int getID() {
        return id;
    }

    /**
     * Sets the current limit for the motor.
     * 
     * @param amps The maximum current in Amperes.
     */
    public void setCurrentLimit(int amps) {
        var config = talon.getConfigurator();
        var currentConfig = new CurrentLimitsConfigs();

        currentConfig.SupplyCurrentLimitEnable = true;
        currentConfig.SupplyCurrentLimit = amps;
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = amps * 1.7;
        config.apply(currentConfig);
    }

    public void setBrakeMode(boolean enabled) {
        talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    protected double uGetVelocity() {
        return talon.getVelocity().getValue();
    }

    public TalonFX withMaxVoltage(double voltage) {
        setMaxVoltage(voltage);
        return this;
    }

    /**
     * Constructor for the Falcon motor controller.
     * 
     * @param deviceNumber The CAN ID for the motor controller.
     * @param isReversed   Indicates whether the motor output should be reversed.
     * @param isStallable  Indicates whether the motor should have stall voltage
     *                     applied.
     */
    public TalonFX(int deviceNumber, boolean isReversed, String bus, boolean isStallable) {
        super(isReversed);

        this.id = deviceNumber;

        this.talon = new com.ctre.phoenix6.hardware.TalonFX(deviceNumber, bus);

        talon.setInverted(false);

        setCurrentLimit(40);
        resetEncoder();
    }

    /**
     * Overloaded constructor for the Falcon motor controller without stallable
     * parameter.
     * 
     * @param deviceNumber The CAN ID for the motor controller.
     * @param isReversed   Indicates whether the motor output should be reversed.
     */
    public TalonFX(int deviceNumber, boolean isReversed) {
        this(deviceNumber, isReversed, "rio", false);
    }

    public TalonFX(int deviceNumber, String bus, boolean isReversed) {
        this(deviceNumber, isReversed, bus, false);
    }

    /**
     * Sets the voltage output of the motor, taking into account stall voltage.
     * 
     * @param volts The desired voltage.
     */
    protected void uSetVoltage(double volts) {
        talon.setVoltage(volts); // Apply the full voltage if above stall level.
    }

    /**
     * Retrieves the number of revolutions from the motor's integrated sensor.
     * 
     * @return The position of the encoder in revolutions.
     */
    protected double uGetRevs() {
        return talon.getPosition().getValue();
    }

    /**
     * Stops the motor immediately by cutting power.
     */
    public void stop() {
        talon.stopMotor();
    }

    public com.ctre.phoenix6.hardware.TalonFX getRawMotor() {
        return talon;
    }
}
