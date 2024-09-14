package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.Core.FieldData;
import frc.Core.Time;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Devices.Imu;
import frc.Devices.LimeLight;
import frc.lib.math.AngleMath;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry odometry;
    public SwerveModule[] mSwerveMods;
    public Imu gyro;
    public Field2d field = new Field2d();

    public Swerve() {
        gyro = new Imu(Constants.Swerve.pigeonID);
        limeLight = new LimeLight(Constants.Swerve.shooterLimeLightID);
        // gyro.setYaw(0);

        field.setRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        Shuffleboard.getTab("SmartDashboard").add(field);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        odometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        FieldData.getIsRed() ? getGyroYaw() : new Rotation2d(getGyroYaw().getRadians() + 2 * Math.PI))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setBrakeMode() {
        mSwerveMods[0].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        mSwerveMods[1].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        mSwerveMods[2].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        mSwerveMods[3].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public void setCoastMode() {
        mSwerveMods[0].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        mSwerveMods[1].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        mSwerveMods[2].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        mSwerveMods[3].mDriveMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }

    public void fromChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        gyro.setYaw(pose.getRotation().getDegrees());
    }

    public Rotation2d getHeading() {
        return odometry.getPoseMeters().getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        gyro.resetYaw();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    private LimeLight limeLight;

    @Override
    public void periodic() {
        field.setRobotPose(getPose());
        odometry.update(getGyroYaw(), getModulePositions());
        // poseEstimator.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Robot Velocity",
                Math.hypot(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond));
        if (FieldData.getIsTeleop()) {
            SmartDashboard.putNumber("Robot X", getPose().getX());
            SmartDashboard.putNumber("Robot Y", getPose().getY());
            SmartDashboard.putNumber("Robot Yaw", getPose().getRotation().getDegrees());
        }
        // if (FieldData.getIsTeleop()) {
        // if (limeLight.botPoseChanged()) {
        // if (limeLight.getPose().getTranslation()
        // .getDistance(poseEstimator.getEstimatedPosition().getTranslation()) <= 1
        // || AngleMath.getDelta(limeLight.getRobotYaw(),
        // poseEstimator.getEstimatedPosition().getRotation().getDegrees()) >= 8)
        // poseEstimator.addVisionMeasurement(limeLight.getPose(),
        // limeLight.getLastReceiveTime());
        // }
        // }
    }
}