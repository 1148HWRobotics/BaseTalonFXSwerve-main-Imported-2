package frc.Components;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Core.FieldData;
import frc.Devices.BinarySensor;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.MotionController;
import frc.lib.util.PDConstant;
import frc.lib.util.PIDController;

public class Carriage extends SubsystemBase {
    public Double startPos = null;
    public TalonFX motor;
    public BinarySensor noteSensor;
    public MotionController controller = new PIDController(new PDConstant(4, 0));
    boolean prepShot = false;
    boolean isFiring = false;
    boolean hasNote = true;

    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }

    public boolean isFiring() {
        return isFiring;
    }

    public Carriage(TalonFX motor, BinarySensor noteSensor) {
        this.motor = motor;
        this.noteSensor = noteSensor;
        motor.setVoltage(0);
        motor.getRawMotor().setNeutralMode(NeutralModeValue.Brake);
    }

    public void intake() {
        if (!hasNote) {
            motor.setVelocity(0.16 * 360);
        } else {

        }

    }

    public void resetMotor() {
        motor.resetEncoder();
    }

    public void runMotor() {
        motor.setVoltage(12);
        if (hasNote) {
            hasNote = false;
        }
    }

    public void stop() {
        motor.setVoltage(0);
    }

    public void intakeSlow() {
        motor.setVoltage(2);
    }

    public void outtakeSlow() {
        motor.setVoltage(-2);
    }

    public void outTake() {
        hasNote = false;
        motor.setVoltage(-12);
        startPos = null;
    }

    public void prepShot() {
        prepShot = true;
    }

    public void unPrepShot() {
        prepShot = false;
    }

    public void shoot() {
        Command shoot = new Command() {
            @Override
            public void initialize() {
                motor.setVoltage(12);
                startPos = null;
            }

            @Override
            public void execute() {

            }

            @Override
            public void end(boolean interrupted) {
                motor.setVoltage(0);
                stop();
            }
        }.withTimeout(0.5);
        shoot.schedule();
        hasNote = false;

    }

    public boolean hasNote() {
        return hasNote;
    }

    @Override
    public void periodic() {
        if (noteSensor.justEnabled()) {
            hasNote = true;
            stop();
        }

        SmartDashboard.putBoolean("Has Note", hasNote);
        // if (motor.getVoltage() >= 11) {
        // isFiring = true;
        // } else
        // isFiring = false;
        // }

    }
}
