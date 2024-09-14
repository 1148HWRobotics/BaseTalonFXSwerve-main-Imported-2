package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.Components.Carriage;
import frc.Components.Elevator;
import frc.Components.Shooter;
import frc.Components.Auto.AutoCommands;
import frc.Core.FieldData;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.PDConstant;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.Vector2;

public final class Constants {
        public static final double stickDeadband = 0.1;

        public static final class Swerve {
                public static final int pigeonID = 18;
                public static final String shooterLimeLightID = "limelight-a";
                public static final String intakeLimeLightID = "limelight-b";

                public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific
                                                                              // robot
                                COTSTalonFXSwerveConstants.SDS.MK4
                                                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L3);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(21.25); // TODO: This must be tuned to
                                                                                     // specific
                                                                                     // robot
                public static final double wheelBase = Units.inchesToMeters(21.25); // TODO: This must be tuned to
                                                                                    // specific
                                                                                    // robot
                public static final double wheelCircumference = chosenModule.wheelCircumference;

                /*
                 * Swerve Kinematics
                 * No need to ever change this unless you are not doing a traditional
                 * rectangular/square 4 module swerve
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

                /* Module Gear Ratios */
                public static final double driveGearRatio = chosenModule.driveGearRatio;
                public static final double angleGearRatio = chosenModule.angleGearRatio;

                /* Motor Inverts */
                public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
                public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

                /* Angle Encoder Invert */
                public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

                /* Swerve Current Limiting */
                public static final int angleCurrentLimit = 35;
                public static final int angleCurrentThreshold = 70;
                public static final double angleCurrentThresholdTime = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveCurrentLimit = 75;
                public static final int driveCurrentThreshold = 70;
                public static final double driveCurrentThresholdTime = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /*
                 * These values are used by the drive falcon to ramp in open loop and closed
                 * loop driving.
                 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
                 */
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                /* Angle Motor PID Values */
                public static final double angleKP = chosenModule.angleKP;
                public static final double angleKI = chosenModule.angleKI;
                public static final double angleKD = chosenModule.angleKD;

                /* Drive Motor PID Values */
                public static final double driveKP = 69.420; // TODO: This must be tuned to specific robot
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.04;
                public static final double driveKF = 0.0;

                /* Drive Motor Characterization Values From SYSID */
                public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
                public static final double driveKV = 1.51;
                public static final double driveKA = 0.27;

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static final double maxSpeed = 5.2; // TODO: This must be tuned to specific robot
                /** Radians per Second */
                public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

                /* Neutral Modes */
                public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
                public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Coast;

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 { // Front Left Module
                        public static final int driveMotorID = 3; // Left Front Go motor ID
                        public static final int angleMotorID = 4; // Left Front Turn motor ID
                        public static final int canCoderID = 23; // Left Front Encoder CAN ID, assuming it acts as the
                                                                 // canCoder for
                                                                 // this module
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(135.96679); // Adjusted to
                                                                                                        // match the
                                                                                                        // left
                                                                                                        // front encoder
                                                                                                        // offset
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                public static final class Mod1 { // Front Right Module
                        public static final int driveMotorID = 5; // Right Front Go motor ID
                        public static final int angleMotorID = 6; // Right Front Turn motor ID
                        public static final int canCoderID = 24; // Right Front Encoder CAN ID, assuming it acts as the
                                                                 // canCoder for
                                                                 // this module
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-9.66796); // Adjusted to
                                                                                                       // match the
                                                                                                       // right front
                                                                                                       // encoder
                                                                                                       // offset
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                public static final class Mod2 { // Back Left Module
                        public static final int driveMotorID = 1; // Left Back Go motor ID
                        public static final int angleMotorID = 2; // Left Back Turn motor ID
                        public static final int canCoderID = 22; // Left Back Encoder CAN ID, assuming it acts as the
                                                                 // canCoder for
                                                                 // this module
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(46.64843); // Adjusted to
                                                                                                       // match the
                                                                                                       // left back
                                                                                                       // encoder offset
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                public static final class Mod3 { // Back Right Module
                        public static final int driveMotorID = 7; // Right Back Go motor ID
                        public static final int angleMotorID = 8; // Right Back Turn motor ID
                        public static final int canCoderID = 21; // Right Back Encoder CAN ID, assuming it acts as the
                                                                 // canCoder for
                                                                 // this module
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(99.93164); // Adjusted to
                                                                                                       // match the
                                                                                                       // right
                                                                                                       // back encoder
                                                                                                       // offset
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }
        }

        public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                                  // tuned to specific robot

                private static PIDConstants translationConstants = new PIDConstants(9.1, 0.008, 0.06);
                private static PIDConstants rotationConstants = new PIDConstants(10.9, 0, 0.07);
                private static ReplanningConfig replanningConfig = new ReplanningConfig();
                private static PDConstant autoAimPDConstants = new PDConstant(0.6, 0.0);
                private static SendableChooser<String> autoChooser = new SendableChooser<String>();

                public static void initializeAutonomous(TalonFX intake, Elevator elevator, Shooter shooter,
                                Carriage carriage, frc.robot.subsystems.Swerve drive) {
                        AutoCommands.initializeAutonomousCommands(intake, elevator, shooter, carriage, drive);
                        autoChooser.setDefaultOption("No Auto", "No Auto");
                        autoChooser.addOption("5 Note Auto",
                                        "5 Note Auton" + (FieldData.getIsRed() ? " Red" : " Blue"));
                        autoChooser.addOption("Long Side Far Notes",
                                        "Right Far Note" + (FieldData.getIsRed() ? " Red" : " Blue"));
                        autoChooser.addOption("Commit Arson", "Commit Arson");
                        autoChooser.addOption("Shoot Preload", "Shoot Preload");
                        autoChooser.addOption("Left Notes", "Left Notes" + (FieldData.getIsRed() ? " Red" : " Blue"));
                }

                public static PathPlannerAuto getAutonToRun() {
                        return new PathPlannerAuto(getAutoSelector().getSelected());
                }

                public static HolonomicPathFollowerConfig getPathFollowerConfig() {
                        return new HolonomicPathFollowerConfig(translationConstants, rotationConstants,
                                        Constants.Swerve.maxSpeed,
                                        Constants.Swerve.trackWidth / 2.0, replanningConfig);
                }

                public static Vector2 getSpeakerPosition() {
                        return new Vector2(FieldData.getIsRed() ? 337.87 : -337.87, 60);
                }

                public static PDConstant getAutoAimPDConstants() {
                        return autoAimPDConstants;
                }

                public static SendableChooser<String> getAutoSelector() {
                        return autoChooser;
                }
        }
}
