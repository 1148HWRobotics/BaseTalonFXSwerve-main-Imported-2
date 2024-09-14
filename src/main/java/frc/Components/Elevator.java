package frc.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Devices.BinarySensor;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.DeSpam;
import frc.lib.util.MathPlus;
import frc.lib.util.MotionController;

public class Elevator extends SubsystemBase {
    TalonFX left;
    TalonFX right;
    public BinarySensor zero;

    public Elevator(TalonFX left, TalonFX right, MotionController constant, BinarySensor zero) {
        this.left = left;
        this.right = right;
        this.zero = zero;
        left.setBrakeMode(true);
        right.setBrakeMode(true);

        left.resetEncoder();
        right.resetEncoder();

        // if (constant instanceof PIDController)
        // ((PIDController) constant).setDeadZone(0.5);

        left.setVelocityPD(constant.clone());
        right.setVelocityPD(constant.clone());

        left.setCurrentLimit(140);
        right.setCurrentLimit(140);
    }

    public boolean isDown() {
        return target != null && target <= 0;
    }

    public Double target = 0.0;

    public double getHeight() {
        return left.getDegrees();
    }

    public double getTarget() {
        if (target == null)
            return 0;
        return target;
    }

    final double upHeight = 360.0 * 23.1;
    final double downHeight = -360 * 0.1;
    final double climbHeight = 360 * 17.1;

    public void moveUp() {
        target = upHeight;
    }

    public void moveToClimb() {
        target = climbHeight;
    }

    public void moveRaw(double input) {
        if (target == null)
            return;
        target = MathPlus.clampVal(target + input, downHeight, upHeight);
    }

    public void moveDown() {
        target = 0.0;
    }

    public void climbDown() {
        target = null;
        left.setVoltage(-12);
        right.setVoltage(-12);
        System.out.println("climbed down");
    }

    public void stretch() {
        target = null;
        left.setVoltage(1);
        right.setVoltage(1);
    }

    public void manualControl(boolean goDown, boolean JoystickMoving) {
        // target = null;
        double vel = 1.8;
        int direction = goDown ? -1 : 1;
        direction = direction * 360;
        if (getHeight() < upHeight && getHeight() > downHeight && JoystickMoving) {
            target = null;
            left.setVelocity(vel);
            right.setVelocity(vel);
            System.out.println("moving");
        } else if (getHeight() > upHeight) {
            moveUp();
        } else if (getHeight() < downHeight) {
            moveDown();
        } else if (target != upHeight && target != downHeight) {
            target = getHeight();
        }
        // if ((getHeight() > upHeight && !goDown) || (getHeight() < downHeight &&
        // goDown)) {
        // pause();
        // } else {
        // left.setVelocity(direction * 3);
        // right.setVelocity(direction * 3);
        // }
    }

    // public void pause() {
    // left.setVelocity(0);
    // right.setVelocity(0);
    // }

    DeSpam dSpam = new DeSpam(0.5);

    @Override
    public void periodic() {
        if (target != null) {
            target = Math.max(Math.min(target, upHeight), downHeight);
            var goingDown = target <= 0;
            final var height = getHeight() + 0;

            var slowFac = 30;
            var max = 48 * slowFac;
            if (!goingDown)
                max *= 1.6;
            var vel = MathPlus.clampAbsVal(target - height, max) / slowFac;

            if (!zero.get() && goingDown) {
                left.resetEncoder();
                right.resetEncoder();
                left.setVoltage(0);
                right.setVoltage(0);
            } else {
                left.setVelocity(vel);
                right.setVelocity(vel);
            }
        }
    }
}
