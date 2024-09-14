package frc.Components.Auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Components.Carriage;
import frc.Components.Elevator;
import frc.Devices.Motor.TalonFX;
import frc.robot.subsystems.Swerve;
import frc.Components.Shooter;
import frc.Core.FieldData;

public class AutoCommands {
    public static void initializeAutonomousCommands(TalonFX intake, Elevator elevator, Shooter shooter,
            Carriage carriage,
            Swerve swerve) {
        if (shooter.isSpinning()) {
            shooter.toggleSpinning();
        }
        NamedCommands.registerCommand("intake", getIntakeCommand(intake, carriage));
        NamedCommands.registerCommand("spinShooter",
                getToggleShooterCommand(carriage, shooter, intake).asProxy());
        NamedCommands.registerCommand("fireShooter",
                getFireShooterCommand(carriage));

    }

    public static Command getIntakeCommand(TalonFX intake, Carriage carriage) {
        return new Command() {
            boolean end = false;

            public void initialize() {
                carriage.setHasNote(false);
                end = false;

            }

            @Override
            public void execute() {
                if (!carriage.noteSensor.justEnabled()) {
                    intake.setVelocity(0.3 * 360);
                    carriage.intake();
                } else if (carriage.noteSensor.justEnabled()) {
                    end = true;
                    intake.setVelocity(0);
                    carriage.stop();
                    this.cancel();
                }
                if (FieldData.getIsTeleop()) {
                    this.cancel();
                }

            }

            @Override
            public boolean isFinished() {
                return end;
            }

            @Override
            public void end(boolean interrupted) {
                intake.setVelocity(0);
                carriage.stop();

            }
        }.withTimeout(3);
    }

    public static Command getToggleShooterCommand(Carriage carriage, Shooter shooter, TalonFX intake) {
        return new Command() {
            boolean end = false;

            public void initialize() {
                shooter.spin();
                end = false;
            }

            @Override
            public void execute() {
                if (shooter.isAtVelocity()) {
                    carriage.shoot();
                    intake.setVoltage(12);
                    new Command() {
                        @Override
                        public void initialize() {
                            intake.setVoltage(12);
                        }

                        @Override
                        public void end(boolean interrupted) {
                            intake.setVoltage(0);
                        }
                    }.withTimeout(0.5).schedule();

                    end = true;
                    this.cancel();
                } else {
                    carriage.stop();
                    intake.setVelocity(0);
                }
                if (FieldData.getIsTeleop()) {
                    this.cancel();
                }
            }

            @Override
            public boolean isFinished() {
                return end;
            }

            @Override
            public void end(boolean interrupted) {

            }

        }.withTimeout(2);
    }

    public static Command getFireShooterCommand(Carriage carriage) {
        return new Command() {
            public void initialize() {

                carriage.shoot();
            }

            @Override
            public void execute() {

            }

            @Override
            public void end(boolean interrupted) {
                carriage.stop();
            }
        };
    }

    public static Command getBaseCommand(Object subsystem) {
        return new Command() {
            public void initialize() {

            }

            @Override
            public void execute() {

            }

            @Override
            public void end(boolean interrupted) {

            }
        };
    }
}
