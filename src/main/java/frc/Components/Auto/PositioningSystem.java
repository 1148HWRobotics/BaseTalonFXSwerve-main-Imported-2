package frc.Components.Auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.util.Vector2;

public interface PositioningSystem {
    double getTurnAngle();

    Vector2 getPosition();

    ChassisSpeeds getRobotRelativeSpeeds();

}
