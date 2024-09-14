package frc.Components;

import frc.Devices.Motor.TalonFX;

public class Lifter {
    TalonFX left;
    TalonFX right;

    public Lifter(TalonFX left, TalonFX right) {
        this.left = left;
        this.right = right;
    }
}
