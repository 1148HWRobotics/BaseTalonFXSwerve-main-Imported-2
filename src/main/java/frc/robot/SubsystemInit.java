package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import frc.Components.Carriage;
import frc.Components.Elevator;
import frc.Components.Shooter;

import frc.Devices.BetterPS4;
import frc.Devices.BinarySensor;
import frc.Devices.Imu;
import frc.Devices.LimeLight;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.PDConstant;
import frc.lib.util.PIDConstant;
import frc.lib.util.PIDController;

public class SubsystemInit {

    static LimeLight shooterLimelight() {
        var limeLightA = new LimeLight("limelight-a");
        limeLightA.setCamMode(true);
        limeLightA.setLEDState(1);
        return limeLightA;
    }

    static LimeLight intakeLimelight() {
        var limeLightB = new LimeLight("limelight-b");
        limeLightB.setCamMode(false);
        limeLightB.setLEDState(1);
        return limeLightB;
    }

    static Imu imu() {
        Imu imu = new Imu(18);
        return imu;
    }

    static Shooter shooter() {
        Shooter shooter = new Shooter(
                new TalonFX(12, false),
                new TalonFX(10, true));
        return shooter;

    }

    static Elevator elevator() {
        BinarySensor elevatorDownSensor = new BinarySensor(0);
        TalonFX f1 = new TalonFX(9, false).withMaxVoltage(12);
        TalonFX f2 = new TalonFX(13, true).withMaxVoltage(12);
        var elevator = new Elevator(
                f1, // left
                f2, // right
                new PIDController(new PIDConstant(0.13, 0.0, 0.0)),
                elevatorDownSensor);
        return elevator;
    }

    static TalonFX intake() {
        var intake = new TalonFX(14, false);
        intake.setVelocityPD(new PIDController(new PDConstant(0.1, 0.0)));
        intake.setCurrentLimit(35);

        return intake;
    }

    static Carriage carriage(BinarySensor intakeSensor) {
        var motor = new TalonFX(11, true);
        motor.setVelocityPD(new PIDController(new PDConstant(0.1, 0.0)));
        motor.setCurrentLimit(40);
        return new Carriage(motor, intakeSensor);
    }

    static BinarySensor intakeSensor() {
        BinarySensor intakeSensor = new BinarySensor(2);
        return intakeSensor;
    }

    // input

    static BetterPS4 con() {
        BetterPS4 con = new BetterPS4(0);
        return con;
    }

    static Joystick joystick() {
        Joystick joystick = new Joystick(1);
        return joystick;
    }

}
