// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import swervelib.SwerveInputStream;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem m_drive =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  // Configure drive input stream
  SwerveInputStream driveInput =
      SwerveInputStream.of(
              m_drive.getSwerveDrive(),
              () -> m_driverController.getLeftY(),
              () -> m_driverController.getLeftX())
          .withControllerRotationAxis(() -> -m_driverController.getRightX())
          .deadband(0.1)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    // Set default drive command
    m_drive.setDefaultCommand(m_drive.driveFieldOriented(driveInput));

        // Set up Auto Factory for Choreo
    autoFactory = new AutoFactory(
            m_drive::getPose, // A function that returns the current robot pose
            m_drive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            m_drive::drive(), // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            m_drive // The drive subsystem
        );
    autoChooser = new AutoChooser();

    //autoChooser.addRoutine("Routine 1", this::routine1);
    //autoChooser.addCmd("Command 1", this::command1);

    SmartDashboard.putData(autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Zero gyro with A button
    m_driverController.a().onTrue(Commands.runOnce(m_drive::zeroGyro));

    // Lock wheels with left bumper
    m_driverController
        .leftBumper()
        .whileTrue(Commands.runOnce(m_drive::lock, m_drive).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return null;
  }
}
