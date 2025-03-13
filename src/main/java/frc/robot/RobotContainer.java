// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public final SwerveSubsystem m_drive =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
  private final ElevatorSubsystem m_elevator =
      Robot.isReal() ? new ElevatorSubsystem() : new ElevatorSubsystemSim();
  private final CoralHandlerSubsystem m_coral =
      Robot.isReal()
          ? new CoralHandlerSubsystem()
          : new CoralHandlerSubsystemSim(m_drive.getSimDrive(), m_elevator);

  // Driver joysticks
  private final FilteredJoystick m_leftJoystick =
      new FilteredJoystick(IOConstants.kLeftJoystickPort);
  private final FilteredJoystick m_rightJoystick =
      new FilteredJoystick(IOConstants.kRightJoystickPort);

  // Operator controller
  private final CommandXboxController m_controller =
      new CommandXboxController(IOConstants.kControllerPort);

  // Button Board
  private final FilteredButton m_buttonBoard = new FilteredButton(IOConstants.kButtonBoardPort);

  public final AutoChooser m_autoChooser = new AutoChooser();
  private final AutoFactory m_autoFactory =
      new AutoFactory(
          m_drive::getPose, m_drive::resetOdometry, m_drive::followTrajectory, true, m_drive);
  private final Routines m_routines = new Routines(m_autoFactory);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    m_autoChooser.addRoutine("Test Routine", m_routines::test);
    m_autoChooser.addRoutine("Blue Processor Routine", m_routines::blueProcessor);
    m_autoChooser.addRoutine("Blue Coral Station Routine", m_routines::blueCoralStation);
    m_autoChooser.addRoutine("Blue Reef K Routine", m_routines::blueCoralToReefK);
    m_autoChooser.addRoutine("Blue Test Full Routine", m_routines::blueTestFull);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    if (IOConstants.kTestMode) {
      m_drive.setDefaultCommand(
          new Drive(
              m_drive,
              m_controller::getLeftX,
              m_controller::getLeftY,
              () -> -m_controller.getRightX(),
              () -> m_controller.rightBumper().getAsBoolean(),
              Optional.empty()));
    } else {
      m_drive.setDefaultCommand(
          new Drive(
              m_drive,
              m_leftJoystick::getX,
              m_leftJoystick::getY,
              m_rightJoystick::getX,
              m_rightJoystick::getButtonTwo,
              Optional.of(m_leftJoystick::getThrottle)));
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
    if (IOConstants.kTestMode) {
      m_controller.a().onTrue(Commands.runOnce(m_drive::zeroGyro));
    } else {

      // Zero gyro with A button
      m_controller.a().onTrue(Commands.runOnce(m_drive::zeroGyro));

      if (!Robot.isReal()) {
        m_controller
            .b()
            .onTrue(Commands.runOnce(() -> ((CoralHandlerSubsystemSim) m_coral).getSimCoral()));
      }

      m_controller
          .x()
          .onTrue(Commands.runOnce(() -> m_elevator.setState(ElevatorSubsystem.ElevatorState.L2)));
      m_controller
          .y()
          .onTrue(
              Commands.runOnce(() -> m_elevator.setState(ElevatorSubsystem.ElevatorState.DOWN)));

      m_controller
          .rightBumper()
          .onTrue(Commands.runOnce(m_coral::grab))
          .onFalse(Commands.runOnce(m_coral::idle));
      m_controller
          .leftBumper()
          .onTrue(Commands.runOnce(m_coral::release))
          .onFalse(Commands.runOnce(m_coral::idle));
    }
  }
}
