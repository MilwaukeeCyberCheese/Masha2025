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
  private final SwerveSubsystem drive =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
  private final ElevatorSubsystem elevator =
      Robot.isReal() ? new ElevatorSubsystem() : new ElevatorSubsystemSim();
  private final CoralHandlerSubsystem coral =
      Robot.isReal()
          ? new CoralHandlerSubsystem()
          : new CoralHandlerSubsystemSim(drive.getSimDrive(), elevator);

  // Driver joysticks
  private final FilteredJoystick driverLeftJoystick =
      new FilteredJoystick(IOConstants.LEFT_JOYSTICK_PORT);
  private final FilteredJoystick driverRightJoystick =
      new FilteredJoystick(IOConstants.RIGHT_JOYSTICK_PORT);

  // Operator controller
  private final CommandXboxController operatorController =
      new CommandXboxController(IOConstants.OPERATOR_CONTROLLER_PORT);

  // Button Board
  private final FilteredButton buttonBoard = new FilteredButton(IOConstants.BUTTON_BOARD_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    // Set default drive command
    if (IOConstants.TEST_MODE) {
      drive.setDefaultCommand(
          new Drive(
                  drive,
              operatorController::getRightX,
              operatorController::getLeftY,
              () -> -operatorController.getRightX(),
              () -> operatorController.rightBumper().getAsBoolean(),
              Optional.empty()));
    } else {
      drive.setDefaultCommand(
          new Drive(
                  drive,
              driverLeftJoystick::getX,
              driverLeftJoystick::getY,
              driverRightJoystick::getX,
              driverRightJoystick::getButtonTwo,
              Optional.of(driverLeftJoystick::getThrottle)));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Test mode allows everything to be run on a single controller
    // Test mode should not be enabled in competition
    if (IOConstants.TEST_MODE) {

    } else {

      // Zero gyro with A button
      operatorController.a().onTrue(Commands.runOnce(drive::zeroGyro));

      if (!Robot.isReal()) {
        operatorController
            .b()
            .onTrue(Commands.runOnce(() -> ((CoralHandlerSubsystemSim) coral).getSimCoral()));
      }

      operatorController
          .x()
          .onTrue(Commands.runOnce(() -> elevator.setState(ElevatorSubsystem.ElevatorState.L2)));
      operatorController
          .y()
          .onTrue(
              Commands.runOnce(() -> elevator.setState(ElevatorSubsystem.ElevatorState.DOWN)));

      operatorController
          .rightBumper()
          .onTrue(Commands.runOnce(coral::grab))
          .onFalse(Commands.runOnce(coral::idle));
      operatorController
          .leftBumper()
          .onTrue(Commands.runOnce(coral::release))
          .onFalse(Commands.runOnce(coral::idle));
    }
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
