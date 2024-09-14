package frc.Devices.Motor;

import frc.Devices.AnyMotor;

public class MotorGroup {
    AnyMotor[] motors;

    public MotorGroup(AnyMotor... motors) {
        this.motors = motors;
    }

    public void setVoltage(double voltage) {
        for (var motor : motors) {
            motor.setVoltage(voltage);
        }
    }

    public void setCurrentLimit(int ampLimit) {
        for (var motor : motors) {
            motor.setCurrentLimit(ampLimit);
        }
    }
}
