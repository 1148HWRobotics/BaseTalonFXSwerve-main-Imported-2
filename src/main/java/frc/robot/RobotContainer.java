package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.Components.Carriage;
import frc.Components.Elevator;
import frc.Components.Shooter;
import frc.Components.Auto.AutoAim;
import frc.Core.FieldData;
import frc.Devices.DriverCamera;
import frc.Devices.LimeLight;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.BetterPS4;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.PoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final BetterPS4 driver = new BetterPS4(0);
    private final Joystick joystick = SubsystemInit.joystick();

    /* Subsystems */
    public final Elevator elevator = SubsystemInit.elevator();
    public final Shooter shooter = SubsystemInit.shooter();
    public final TalonFX intake = SubsystemInit.intake();
    public final Carriage carriage = SubsystemInit.carriage(SubsystemInit.intakeSensor());
    public final Swerve s_Swerve = new Swerve();
    public final PoseEstimator estimator = new PoseEstimator(s_Swerve);
    private final AutoAim autoAim = new AutoAim(s_Swerve);
    private final DriverCamera driverCamera = new DriverCamera();

    /* Driver Buttons */
    private final JoystickButton moveElevator = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton outTakeButton = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton shooterButton = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton fireButton = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final POVButton moveElevatorDownRaw = new POVButton(joystick, 180);
    private final POVButton moveElevatorUpRaw = new POVButton(joystick, 0);
    private final JoystickButton climbDownHard = new JoystickButton(joystick, 3);
    private final JoystickButton moveToClimb = new JoystickButton(joystick, 5);
    private final JoystickButton stretchElevator = new JoystickButton(joystick, 6);
    private final JoystickButton intakeSlow = new JoystickButton(joystick, 12);
    private final JoystickButton outTakeSlow = new JoystickButton(joystick, 10);
    private final JoystickButton spinShooterOperator = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);
    private final JoystickButton driverIsFOC = new JoystickButton(driver, PS4Controller.Button.kCircle.value);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        elevator, shooter, intake, carriage, autoAim, driverIsFOC));

        AutoBuilder.configureHolonomic(
                s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::getRobotVelocity,
                s_Swerve::fromChassisSpeeds,
                Constants.AutoConstants.getPathFollowerConfig(),
                FieldData::getIsRed, s_Swerve);

        // Configure the button bindings
        // System.out.println("im here");
        configureButtonBindings();

        Shuffleboard.getTab("SmartDashboard").add("Auto Chooser", Constants.AutoConstants.getAutoSelector());
        SmartDashboard.putBoolean("Field-Oriented Control", true);
        SmartDashboard.putData("Reset FOC", new Command() {
            @Override
            public void initialize() {
                s_Swerve.zeroHeading();
            }
        });
        SmartDashboard.putData("Flip FOC", new Command() {
            @Override
            public void initialize() {
                s_Swerve.setHeading(new Rotation2d(s_Swerve.getHeading().getRadians() + Math.PI));
            }
        });

        // SmartDashboard.putData("Auto Chooser",
        // Constants.AutoConstants.getAutoSelector());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        outTakeSlow.whileTrue(new InstantCommand(() -> {
            carriage.outtakeSlow();
        })).onFalse(new InstantCommand(() -> {
            carriage.stop();
        }));
        intakeSlow.whileTrue(new InstantCommand(() -> {
            carriage.intakeSlow();
        })).onFalse(new InstantCommand(() -> {
            carriage.stop();
        }));
        ;
        moveElevatorDownRaw.whileTrue(new Command() {
            public void execute() {
                elevator.moveRaw(-0.2 * 360);
            }
        }).onFalse(new Command() {
            public void execute() {
                elevator.moveRaw(0);
            }
        }.withTimeout(0.02));
        moveElevatorUpRaw.whileTrue(new Command() {
            @Override
            public void execute() {
                elevator.moveRaw(0.2 * 360);
            }
        }).onFalse(new Command() {
            @Override
            public void execute() {
                elevator.moveRaw(0);
            }
        }.withTimeout(0.02));
        /* Driver Buttons */
        intakeButton.whileTrue(new Command() {
            @Override
            public void execute() {
                if (carriage.hasNote()) {

                } else if (!carriage.hasNote() && elevator.isDown()) {
                    intake.setVelocity(0.4 * 360);
                    carriage.intake();
                } else if (!carriage.hasNote() && !elevator.isDown()) {
                    carriage.intake();
                } else if (!carriage.noteSensor.get()) {
                    intake.setVelocity(0);
                    carriage.stop();
                }
            }
        }).onFalse(new InstantCommand(() -> {
            if (intakeSlow.getAsBoolean() || outTakeSlow.getAsBoolean()) {

            } else {
                carriage.stop();
                intake.setVelocity(0);
            }

        }));
        moveElevator.onTrue(new InstantCommand(() -> {
            if (elevator.isDown())
                elevator.moveUp();
            else
                elevator.moveDown();
        }));

        moveToClimb.onTrue(new InstantCommand(() -> elevator.moveToClimb()));
        climbDownHard.onTrue(new InstantCommand(() -> elevator.climbDown()));
        stretchElevator.onTrue(new InstantCommand(() -> elevator.stretch()));
        outTakeButton.onTrue(new InstantCommand(() -> {
            carriage.setHasNote(false);
        })).onFalse(new InstantCommand(() -> {
            carriage.setHasNote(false);
        }));
        outTakeButton.whileTrue(new InstantCommand(() -> {
            carriage.outTake();
            intake.setVoltage(-12);
        })).onFalse(new InstantCommand(() -> {
            if (intakeSlow.getAsBoolean() || outTakeSlow.getAsBoolean()) {

            } else {
                carriage.stop();
                intake.setVelocity(0);
            }
        }));
        shooterButton.onTrue(new InstantCommand(() -> {
            if (elevator.isDown()) {
                shooter.toggleSpinning();
                if (shooter.isSpinning()) {
                    carriage.prepShot();
                    autoAim.setIsAutoAimOn(true);
                } else {
                    carriage.unPrepShot();
                    autoAim.setIsAutoAimOn(false);
                }
            } else if (shooter.isSpinning()) {
                shooter.toggleSpinning();
            }
        }));
        // spinShooterOperator.onTrue(new InstantCommand(() -> {
        // if (elevator.isDown()) {
        // shooter.toggleSpinning();
        // if (shooter.isSpinning()) {
        // carriage.prepShot();
        // autoAim.setIsAutoAimOn(true);
        // } else {
        // carriage.unPrepShot();
        // autoAim.setIsAutoAimOn(false);
        // }
        // }
        // }));
        fireButton.onTrue(new InstantCommand(() -> {
            if (shooter.isSpinning() && elevator.isDown()) {
                carriage.shoot();
            } else if (elevator.zero.get() || elevator.isDown()) {
                carriage.outTake();
            }
        })).onFalse(new InstantCommand(() -> {
            carriage.stop();
        }));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto(Constants.AutoConstants.getAutoSelector().getSelected());
    }

}
