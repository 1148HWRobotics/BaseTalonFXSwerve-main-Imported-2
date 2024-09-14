package frc.Devices;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.lib.math.AngleMath;

/**
 * The AbsoluteEncoder class is used to interface with an absolute encoder
 * connected via CAN.
 */
public class AbsoluteEncoder {
    CANcoder coder; // The CANcoder device representing the absolute encoder.
    double zeroReading; // The encoder reading considered as the zero position.
    boolean reversed = false; // Flag indicating if the encoder values should be reversed.

    /**
     * Reverses the encoder reading if the reversed flag is set.
     * 
     * @param num The encoder reading.
     * @return The encoder reading, potentially reversed.
     */
    private double reverse(double num) {
        return reversed ? -num : num;
    }

    /**
     * Constructor for an encoder on a specified CAN port, defaults to non-reversed.
     * 
     * @param canPort The CAN port where the encoder is connected.
     */
    public AbsoluteEncoder(int canPort) {
        this(canPort, "rio", false);
    }

    /**
     * Constructor for an encoder with a specified CAN port and reverse flag.
     * 
     * @param canPort    The CAN port where the encoder is connected.
     * @param isReversed Flag to reverse encoder values.
     */
    public AbsoluteEncoder(int canPort, String bus, boolean isReversed) {
        this(canPort, bus, 0, isReversed);
    }

    /**
     * Applies an offset to the zero reading of the encoder.
     * 
     * @param offset The offset in degrees to be applied.
     * @return The AbsoluteEncoder instance for chaining.
     */
    public AbsoluteEncoder setOffset(double offset) {
        zeroReading = AngleMath.conformAngle(zeroReading - offset);
        return this;
    }

    /**
     * Constructor for an encoder with a specified CAN port, zero reading, and
     * reverse flag.
     * 
     * @param canPort     The CAN port where the encoder is connected.
     * @param zeroReading The encoder reading considered as zero.
     * @param isReversed  Flag to reverse encoder values.
     */
    public AbsoluteEncoder(int canPort, String bus, double zeroReading, boolean isReversed) {
        this.reversed = isReversed;
        this.zeroReading = zeroReading;

        // Initialize the CANcoder with the specified CAN port.
        this.coder = new CANcoder(canPort, bus);
        // Set up the encoder configuration.
        var configs = new CANcoderConfiguration();
        // Configure the encoder to measure the absolute position within a +/-180 degree
        // range.
        configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // Apply the configuration to the encoder.
        coder.getConfigurator().apply(configs);
    }

    /**
     * Retrieves the absolute position value from the encoder in degrees, ranging
     * from -180 to 180.
     * 
     * @return The encoder's absolute position, adjusted by the zero reading and
     *         reversed if necessary.
     */
    public double absVal() {
        // Get the absolute position from the encoder, adjust for zero reading and
        // reverse if needed.
        // The returned value is conformed to be within the range of -180 to 180
        // degrees.
        return AngleMath.conformAngle(reverse(coder.getAbsolutePosition().getValue() * 360.0 - zeroReading));
    }
}
