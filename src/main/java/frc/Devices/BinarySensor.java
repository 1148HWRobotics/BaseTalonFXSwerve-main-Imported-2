package frc.Devices;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BinarySensor extends SubsystemBase {
    DigitalInput input;
    Boolean lastState = false;
    boolean justChanged = false;
    final boolean reversed;
    boolean s;

    public BinarySensor(int input, boolean reversed) {
        this.input = new DigitalInput(input);
        this.reversed = reversed;
        if (reversed)
            lastState = true;
        s = this.input.get();
    }

    public BinarySensor(int input) {
        this(input, false);
    }

    public boolean get() {
        if (reversed)
            return !input.get();
        else
            return input.get();
    }

    public boolean justChanged() {
        return justChanged;
    }

    public boolean justDisabled() {
        return justChanged() && !get();
    }

    public boolean justEnabled() {
        return justChanged() && get();
    }

    @Override
    public void periodic() {
        justChanged = lastState != s;
        lastState = s;
        s = get();

    }
}
