package frc.Components.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.AngleMath;
import frc.lib.math.Conversions;
import frc.lib.util.Container;
import frc.lib.util.PIDConstant;
import frc.lib.util.PIDController;
import frc.lib.util.Vector2;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoAim extends SubsystemBase {
    private Swerve s_Swerve;
    private PIDController turnPD = new PIDController(Constants.AutoConstants.getAutoAimPDConstants());
    private final Container<Boolean> isAutoAimOn = new Container<>(false);

    private final Vector2 displacementFromTar;
    private double correction;
    public Command cancelAutoAim;

    public AutoAim(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        SmartDashboard.putBoolean("Auto Aim", true);
        displacementFromTar = Constants.AutoConstants.getSpeakerPosition()
                .minus(Conversions.pose2dToPosition(s_Swerve.getPose()).position);
    }

    public double getCorrection() {
        return correction;
    }

    public void updatePID(PIDConstant newConstant) {
        turnPD.setConstant(newConstant);
    }

    public void setIsAutoAimOn(boolean isOn) {
        this.isAutoAimOn.val = isOn;
    }

    public boolean getIsAutoAimOn() {
        return isAutoAimOn.val;
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean("Auto Aim", true)) {
            if (isAutoAimOn.val) {
                correction = turnPD.solve(AngleMath.getDelta(displacementFromTar.getTurnAngleDeg() + 180,
                        s_Swerve.getHeading().getDegrees()));
            } else {
                correction = 0.0;
            }
        } else {
            correction = 0.0;
        }

    }
}
