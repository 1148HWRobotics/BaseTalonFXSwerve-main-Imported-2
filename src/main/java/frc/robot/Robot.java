// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Core.FieldData;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.carriage.resetMotor();
    Constants.AutoConstants.initializeAutonomous(m_robotContainer.intake, m_robotContainer.elevator,
        m_robotContainer.shooter,
        m_robotContainer.carriage, m_robotContainer.s_Swerve);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.s_Swerve.setBrakeMode();

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.shooter.spin();
    m_robotContainer.carriage.resetMotor();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.shooter.setDefaultCommand(m_robotContainer.shooter.run(() -> m_robotContainer.shooter.spin()));
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.s_Swerve.field.setRobotPose(
        PathPlannerAuto.getStaringPoseFromAutoFile(Constants.AutoConstants.getAutoSelector().getSelected()));
    PathPlannerLogging
        .setLogActivePathCallback((poses) -> m_robotContainer.s_Swerve.field.getObject("path").setPoses(poses));

  }

  @Override
  public void autonomousExit() {
    m_robotContainer.shooter.removeDefaultCommand();
    m_robotContainer.shooter.stop();
    if (m_robotContainer.carriage != null) {
      m_robotContainer.carriage.removeDefaultCommand();
    }
    m_robotContainer.s_Swerve.setCoastMode();
  }

  @Override
  public void teleopExit() {
    m_robotContainer.s_Swerve.setBrakeMode();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // m_robotContainer.carriage.resetMotor();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_robotContainer.shooter.stop();
    }
    // if (m_robotContainer.shooter.isSpinning()) {
    // m_robotContainer.shooter.toggleSpinning();
    // }
    // i}
    if (!FieldData.getMatchType().equals("Qualification") || !FieldData.getMatchType().equals("Elimination")) {
      m_robotContainer.s_Swerve.setCoastMode();
    }
    // m_robotContainer.s_Swerve.setCoastMode();
    // if (FieldData.getIsRed()) {
    // m_robotContainer.s_Swerve
    // .setHeading(new
    // Rotation2d(m_robotContainer.s_Swerve.getHeading().getRadians() + Math.PI));
    // }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_robotContainer.carriage.noteSensor.justEnabled()) {
      m_robotContainer.carriage.stop();
      m_robotContainer.intake.setVelocity(0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
