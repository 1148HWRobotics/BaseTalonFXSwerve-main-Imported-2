package frc.Components.Auto;

import java.util.LinkedList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Core.Time;
import frc.Devices.Imu;
import frc.Devices.LimeLight;
import frc.lib.util.Vector2;
import frc.robot.Constants.Swerve;

public class FieldPositioning extends SubsystemBase implements PositioningSystem {
    Swerve drive;
    Imu imu;
    LimeLight limeLight;
    final double correctionTime = 0.5;

    boolean hasGottenLimeLightFrame = false;

    public boolean hasGottenLimeLightFrame() {
        return hasGottenLimeLightFrame;
    }

    public FieldPositioning(Swerve drive, Imu imu, LimeLight limeLight, Position startPos) {
        this.drive = drive;
        this.imu = imu;
        this.limeLight = limeLight;
        positionHistory.add(0, startPos);
    }

    public void setStartPosition(Position position) {
        positionHistory.set(0, position);
    }

    double lastLimelightFrameTime = Double.NEGATIVE_INFINITY;
    Position lastLimelightFrameOffset = new Position(0, new Vector2(0, 0));
    LinkedList<Position> positionHistory = new LinkedList<>();

    private boolean isRationalLimelightFrame() {
        final boolean isAllZero = limeLight.getRobotX() == 0 && limeLight.getRobotY() == 0
                && limeLight.getRobotZ() == 0;
        return limeLight.botPoseChanged() && !isAllZero;
    }

    @Override
    public double getTurnAngle() {
        return positionHistory.getFirst().angle;
    }

    @Override
    public Vector2 getPosition() {
        return positionHistory.getFirst().position;
    }

    public Position predictedPositionAtLastLimelightFrame() {
        // long currentTimeSinceEpoch = System.currentTimeMillis();
        // long receiveTimeSinceEpoch = limeLight.getLastReceiveTime();
        double latencyImageTakenToNow = limeLight.getRobotLatency() / 1000;

        int predictedPositionIndex = (int) (latencyImageTakenToNow / 0.02);

        if (predictedPositionIndex >= positionHistory.size()) {
            return null;
        }

        return positionHistory.get(predictedPositionIndex);

    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        double rotationSpeed = (positionHistory.get(0).angle - positionHistory.get(1).angle) / 0.02;
        Vector2 translationSpeed = (positionHistory.get(0).position.minus(positionHistory.get(1).position))
                .multiply(1 / 0.02).rotate(-getTurnAngle());

        return new ChassisSpeeds(translationSpeed.x, translationSpeed.y, rotationSpeed / 180 * Math.PI);
    }

    public Vector2 getFieldRelativeSpeed() {
        return positionHistory.getFirst().position.minus(positionHistory.get(1).position).multiply(1 / 0.02);
    }

    @Override
    public void periodic() {
        Position lastPosition = positionHistory.getFirst();
        final double currentAngle = lastPosition.angle + imu.getYawDeltaThisTick();
        positionHistory.add(0, new Position(
                currentAngle,
                lastPosition.position.add(new Vector2(0, 0))));// todo Changed to empty vector until further
                                                               // implementation

        // makes sure position history doesn't get too long
        if (positionHistory.size() > 5 / 0.02) {
            positionHistory.removeLast();
        }

        if (isRationalLimelightFrame()) {
            hasGottenLimeLightFrame = true;
            final double timeSinceLastFrame = Time.getTimeSincePower() - lastLimelightFrameTime;
            lastLimelightFrameTime = Time.getTimeSincePower();
            final Position limelightPositionAtFrame = limeLight.getRobotPosition();
            Position predictedPositionAtFrame = predictedPositionAtLastLimelightFrame();
            if (predictedPositionAtFrame == null)
                predictedPositionAtFrame = positionHistory.getFirst();

            Position adjustedPosition;
            // if the time since the last reading is more than 20 ms, we scrub the
            // extrapolated position from the drive
            if (timeSinceLastFrame < 20) {
                adjustedPosition = limelightPositionAtFrame.combine(predictedPositionAtFrame, 0.5);
            } else {
                adjustedPosition = limelightPositionAtFrame;
            }
            Position offsetPosition = adjustedPosition.difference(predictedPositionAtFrame);
            positionHistory.replaceAll(e -> e.add(offsetPosition));
        }
    }
}