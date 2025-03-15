// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.drive.Drive;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.sim.CoralHandlerSubsystemSim;
import frc.robot.subsystems.sim.ElevatorSubsystemSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.FilteredButton;
import frc.robot.utils.FilteredJoystick;
import java.io.File;
import java.util.Optional;

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

  // Driver joysticks
  private final FilteredJoystick m_driverLeftJoystick =
      new FilteredJoystick(IOConstants.kLeftJoystickPort);
  private final FilteredJoystick m_driverRightJoystick =
      new FilteredJoystick(IOConstants.kRightJoystickPort);

  // Operator controller
  private final CommandXboxController m_operatorController =
      new CommandXboxController(IOConstants.kOperatorControllerPort);

  // Button Board
  private final FilteredButton m_buttonBoard = new FilteredButton(IOConstants.kButtonBoardPort);

  private ControllerState controllerState = ControllerState.UNKNOWN;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.updateControllerConnections();
    configureButtonBindings();

    // Set default drive command
    m_drive.setDefaultCommand(new Drive(
            m_drive,
            this::getControllerX,
            this::getControllerY,
            this::getContollerRotation,
            this::getControllerSlow,
            Optional.of(this::getControllerThrottle)
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_operatorController.start().onTrue(Commands.runOnce(() -> this.controllerState = ControllerState.XBOX));
    new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), m_driverLeftJoystick::getButtonNine)
            .onTrue(Commands.runOnce(() -> this.controllerState = ControllerState.JOYSTICKS));
    new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), m_driverRightJoystick::getButtonNine)
            .onTrue(Commands.runOnce(() -> this.controllerState = ControllerState.JOYSTICKS));

    // Zero gyro with A button
    m_operatorController.a().onTrue(Commands.runOnce(m_drive::zeroGyro));

    if (!Robot.isReal()) {
      m_operatorController
          .b()
          .onTrue(Commands.runOnce(() -> ((CoralHandlerSubsystemSim) m_coral).getSimCoral()));
    }

    m_operatorController
        .x()
        .onTrue(Commands.runOnce(() -> m_elevator.setState(ElevatorSubsystem.ElevatorState.L2)));
    m_operatorController
        .y()
        .onTrue(
            Commands.runOnce(() -> m_elevator.setState(ElevatorSubsystem.ElevatorState.DOWN)));

    m_operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(m_coral::grab))
        .onFalse(Commands.runOnce(m_coral::idle));
    m_operatorController
        .leftBumper()
        .onTrue(Commands.runOnce(m_coral::release))
        .onFalse(Commands.runOnce(m_coral::idle));
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

  public boolean isXboxController() {
    return this.controllerState != ControllerState.JOYSTICKS;
  }

  public double getControllerX() {
    if (this.isXboxController()) {
      return m_operatorController.getLeftX();
    } else {
      return m_driverLeftJoystick.getX();
    }
  }

  public double getControllerY() {
    if (this.isXboxController()) {
      return m_operatorController.getLeftY();
    } else {
      return m_driverLeftJoystick.getY();
    }
  }

  public double getContollerRotation() {
    if (this.isXboxController()) {
      return -m_operatorController.getRightX();
    } else {
      return m_driverRightJoystick.getX();
    }
  }

  public boolean getControllerSlow() {
    if (this.isXboxController()) {
      return m_operatorController.rightBumper().getAsBoolean();
    } else {
      return m_driverRightJoystick.getButtonTwo();
    }
  }

  public double getControllerThrottle() {
    if (this.isXboxController()) {
      return 1.0;
    } else {
      return m_driverLeftJoystick.getThrottle();
    }
  }

  private boolean isAnyJoystickConnected() {
    return this.m_driverLeftJoystick.getJoystick().isConnected() || this.m_driverRightJoystick.getJoystick().isConnected();
  }

  public void updateControllerConnections() {
    if (this.controllerState == ControllerState.XBOX && !this.m_operatorController.isConnected()) {
      this.controllerState = ControllerState.UNKNOWN;
    } else if (this.controllerState == ControllerState.JOYSTICKS && !this.isAnyJoystickConnected()) {
      this.controllerState = ControllerState.UNKNOWN;
    }

    if (this.controllerState == ControllerState.UNKNOWN) {
      if (this.m_operatorController.isConnected()) this.controllerState = ControllerState.XBOX;
      else if (this.isAnyJoystickConnected()) this.controllerState = ControllerState.JOYSTICKS;
    }
  }

  private enum ControllerState {
    UNKNOWN,
    XBOX,
    JOYSTICKS,
  }
}
