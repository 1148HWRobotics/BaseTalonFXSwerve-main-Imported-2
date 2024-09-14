package frc.Devices;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Vector2;
import frc.Components.Auto.Position;

public class LimeLight extends SubsystemBase {
    private NetworkTable masterTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry botpose;
    private boolean camEnabled;

    // NOTE: this assumes the limelight is called "limelight" which is not true for
    // every limelight!
    public LimeLight() {
        this("limelight");
    }

    public LimeLight(String limelightHostname) {
        masterTable = NetworkTableInstance.getDefault().getTable(limelightHostname);
        tx = masterTable.getEntry("tx");
        ty = masterTable.getEntry("ty");
        ta = masterTable.getEntry("ta");
        tv = masterTable.getEntry("tv");
        botpose = masterTable.getEntry("botpose");

        camEnabled = true;
    }

    public void setLEDState(int function) {
        // 0 is default, 1 is off, 2 is blink, 3 is on
        masterTable.getEntry("ledMode").setNumber(function);
    }

    boolean botPoseChanged = false;
    long lastBotPoseChangeTime = 0;

    public boolean botPoseChanged() {
        return botPoseChanged;
    }

    public void periodic() {
        if (botpose.getLastChange() != lastBotPoseChangeTime) {
            botPoseChanged = true;
            lastBotPoseChangeTime = botpose.getLastChange();
        } else {
            botPoseChanged = false;
        }
    }

    public Pose2d getPose() {
        return new Pose2d(new Translation2d(getRobotX(), getRobotY()), new Rotation2d(getRobotYaw()));
    }

    public Pose3d getPose3d() {
        return new Pose3d(new Translation3d(getRobotX(), getRobotY(), getRobotZ()),
                new Rotation3d(getRobotRoll(), getRobotPitch(), getRobotYaw()));
    }

    public boolean getCamMode() {
        return camEnabled;
    }

    // true - Vision Processing, false - DriverCam
    public void setCamMode(boolean camMode) {
        masterTable.getEntry("camMode").setValue(camMode ? 0 : 1);
        camEnabled = camMode;
    }

    // read values periodically
    public double getHorixDegreeToTarget() {
        double x = tx.getDouble(0.0);
        return x;
    }

    public Position getRobotPosition() {
        double x = getRobotX();
        double y = getRobotY();
        double angle = getRobotYaw();
        return new Position(angle, new Vector2(x, y));
    }

    // The coordinate system is as follows:
    // x = 0 is at the center of the field. The red side is at positive x and the
    // blue side is at negative x.
    // y = 0 is at the center of the field. The speakers are at positive y.
    // NOTE: these coordinates are in meters and should be appropriately converted
    // before use.
    // Following are the coordinates of the april tags (the z axis represents
    // height):
    // 1. (6.808597, -3.859403, 1.355852)
    // 2. (7.914259, -3.221609, 1.355852)
    // 3. (8.308467, 0.877443, 1.451102)
    // 4. (8.308467, 1.442593, 1.451102)
    // 5. (6.429883, 4.098925, 1.355852)
    // 6. (-6.429375, 4.098925, 1.355852)
    // 7. (-8.308975, 1.442593, 1.451102)
    // 8. (-8.308975, 0.877443, 1.451102)
    // 9. (-7.914767, -3.221609, 1.355852)
    // 10. (-6.809359, -3.859403, 1.355852)
    // 11. (3.633851, -0.392049, 1.3208)
    // 12. (3.633851, 0.393065, 1.3208)
    // 13. (2.949321, -0.000127, 1.3208)
    // 14. (-2.950083, -0.000127, 1.3208)
    // 15. (-3.629533, 0.393065, 1.3208)
    // 16. (-3.629533, -0.392049, 1.3208)
    public double getRobotX() {
        double[] botposeArray = botpose.getDoubleArray(new double[100]);
        return Units.metersToInches(botposeArray[0]);
    }

    public double getRobotY() {
        double[] botposeArray = botpose.getDoubleArray(new double[100]);
        return Units.metersToInches(botposeArray[1]);
    }

    public double getRobotZ() {
        double[] botposeArray = botpose.getDoubleArray(new double[100]);
        return Units.metersToInches(botposeArray[2]);
    }

    public double getRobotRoll() {
        double[] botposeArray = botpose.getDoubleArray(new double[100]);
        return botposeArray[3];
    }

    public double getRobotPitch() {
        double[] botposeArray = botpose.getDoubleArray(new double[100]);
        return botposeArray[4];
    }

    public double getRobotYaw() {
        double[] botposeArray = botpose.getDoubleArray(new double[100]);
        return botposeArray[5];
    }

    public double getRobotLatency() {
        double[] botposeArray = botpose.getDoubleArray(new double[100]);
        return botposeArray[6];
    }

    public long getLastReceiveTime() {
        long lastReceiveTime = botpose.getLastChange();
        return lastReceiveTime;
    }

    public double getVerticalD() {
        double y = ty.getDouble(0.0);
        return y;
    }

    public double getArea() {
        double area = ta.getDouble(0.0);
        return area;
    }

    public boolean foundTarget() {
        return tv.getDouble(0) == 1;
    }
}
