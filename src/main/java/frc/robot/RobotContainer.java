// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.sim.CoralHandlerSubsystemSim;
import frc.robot.subsystems.sim.ElevatorSubsystemSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
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
  private final ElevatorSubsystem m_elevator =
      Robot.isReal() ? new ElevatorSubsystem() : new ElevatorSubsystemSim();
  private final CoralHandlerSubsystem m_coral =
      Robot.isReal()
          ? new CoralHandlerSubsystem()
          : new CoralHandlerSubsystemSim(m_drive.getSimDrive(), m_elevator);

  // Operator controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  // Configure drive input stream
  SwerveInputStream driveInput =
      SwerveInputStream.of(
              m_drive.getSwerveDrive(),
              () ->
                  m_driverController.getLeftY()
                      * DriveConstants.kDrivingSpeed[
                          m_driverController.leftBumper().getAsBoolean() ? 1 : 0],
              () ->
                  m_driverController.getLeftX()
                      * DriveConstants.kDrivingSpeed[
                          m_driverController.leftBumper().getAsBoolean() ? 1 : 0])
          .withControllerRotationAxis(
              () ->
                  -m_driverController.getRightX()
                      * DriveConstants.kRotationSpeed[
                          m_driverController.leftBumper().getAsBoolean() ? 1 : 0])
          .deadband(0.1)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    // Set default drive command
    m_drive.setDefaultCommand(m_drive.driveFieldOriented(driveInput));
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

    // TODO: x-mode

    if (!Robot.isReal()) {
      m_driverController
          .b()
          .onTrue(Commands.runOnce(() -> ((CoralHandlerSubsystemSim) m_coral).getSimCoral()));
    }

    // m_driverController
    //     .x()
    //     .onTrue(Commands.runOnce(() -> m_elevator.setState(ElevatorSubsystem.ElevatorState.L2)));
    // m_driverController
    //     .y()
    //     .onTrue(Commands.runOnce(() ->
    // m_elevator.setState(ElevatorSubsystem.ElevatorState.DOWN)));

    m_operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(m_coral::grab))
        .onFalse(Commands.runOnce(m_coral::idle));
    m_operatorController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> m_coral.setSpeed(.7)))
        .onFalse(Commands.runOnce(() -> m_coral.setSpeed(0)));
    
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
