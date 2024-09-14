package frc.Components;

import javax.naming.InitialContext;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Core.FieldData;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.MathPlus;
import frc.lib.util.MotionController;
import frc.lib.util.PWIDConstant;
import frc.lib.util.PWIDController;

public class Shooter extends SubsystemBase {
    TalonFX left;
    TalonFX right;
    double kP;
    double kI;
    double kD;

    boolean isSpinning = false;
    boolean isSpinningReverse = false;

    public Shooter(TalonFX left, TalonFX right) {
        kP = 0.17;
        kI = 0.001;
        kD = 0.0001;
        SmartDashboard.putNumber("Shooter P Constant", kP);
        SmartDashboard.putNumber("Shooter I Constant", kI);
        SmartDashboard.putNumber("Shooter D Constant", kD);

        this.left = left;
        this.right = right;

        Shuffleboard.getTab("SmartDashboard").add("Reverse Shooter", new Command() {
            @Override
            public void initialize() {
                isSpinningReverse = true;
            }

            @Override
            public void end(boolean interrupted) {
                isSpinningReverse = false;
            }
        }.withTimeout(3));
        // in init function
        // var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        // var slot0Configs = talonFXConfigs.Slot0;
        // slot0Configs.kS = 0.15; // Add 0.25 V output to overcome static friction
        // slot0Configs.kV = 0.06; // A velocity target of 1 rps results in 0.12 V
        // output
        // slot0Configs.kA = 0.1; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0Configs.kP = 0.24; // A position error of 2.5 rotations results in 12 V
        // output
        // slot0Configs.kI = 0.015; // no output for integrated error
        // slot0Configs.kD = 0.01; // A velocity error of 1 rps results in 0.1 V output

        // // set Motion Magic settings
        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity
        // of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of
        // 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s
        // (0.1 seconds)

        // left.getRawMotor().getConfigurator().apply(talonFXConfigs);
        // right.getRawMotor().getConfigurator().apply(talonFXConfigs);
    }

    public void spin() {
        isSpinning = true;
    }

    public void stop() {
        isSpinning = false;
    }

    public void toggleSpinning() {
        isSpinning = !isSpinning;
    }

    public boolean isSpinning() {
        return isSpinning;
    }

    public boolean isSpinningReverse() {
        return isSpinningReverse;
    }

    public boolean isAtVelocity() {
        return MathPlus.withinBounds(left.getVelocity(), vel + 12, vel - 12);
    }

    public double vel = 75;

    private MotionController con = new PWIDController(new PWIDConstant(kP, kD, kI, 1));

    public double getVoltage(double velocity) {
        return vel * 0.12;
    }

    public void periodic() {

        if (isSpinning || isSpinningReverse) {
            // adds the output of the controller to the predicted
            // voltage required to reach the velocity
            double correct = getVoltage((isSpinningReverse ? -vel : vel))
                    + con.solve((isSpinningReverse ? -vel : vel) - left.getVelocity(), 0.02);
            System.out.println("vel: " + left.getVelocity() + " target vel: " + vel + " voltage: " + correct);
            left.setVoltage(correct);
            right.setVoltage(correct);
            // left.getRawMotor().setControl(new MotionMagicVelocityVoltage(70.0));
            // right.getRawMotor().setControl(new MotionMagicVelocityVoltage(70.0));
        } else {
            left.setVoltage(0);
            right.setVoltage(0);
            // left.getRawMotor().setControl(new MotionMagicVelocityVoltage(0.0));
            // right.getRawMotor().setControl(new MotionMagicVelocityVoltage(0.0));
        }
        SmartDashboard.putNumber("Shooter RPM", left.getVelocity());
        SmartDashboard.putBoolean("Is Shooter Spinning", isSpinning);
        con = new PWIDController(new PWIDConstant(kP, kD, kI, 0.6));
        kP = SmartDashboard.getNumber("Shooter P Constant", kP);
        kI = SmartDashboard.getNumber("Shooter I Constant", kI);
        kD = SmartDashboard.getNumber("Shooter D Constant", kD);

    }
    /*
     * kP = 0.22;
     * kI = 0.065;
     * kD = 0.0001;
     */
}
