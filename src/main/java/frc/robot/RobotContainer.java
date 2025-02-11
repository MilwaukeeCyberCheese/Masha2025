// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.sim.CoralHandlerSubsystemSim;
import frc.robot.subsystems.sim.ElevatorSubsystemSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.FilteredButton;
import frc.robot.utils.FilteredJoystick;
import java.io.File;
import swervelib.SwerveInputStream;

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
  private final FilteredJoystick m_driverLeftJoystick =
      new FilteredJoystick(OIConstants.kLeftJoystickPort);
  private final FilteredJoystick m_driverRightJoystick =
      new FilteredJoystick(OIConstants.kRightJoystickPort);

  // Operator controller
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  // Button Board
  private final FilteredButton m_buttonBoard = new FilteredButton(OIConstants.kButtonBoardPort);

  public final AutoChooser m_autoChooser = new AutoChooser();
  private final AutoFactory m_autoFactory =
      new AutoFactory(
          m_drive::getPose, m_drive::resetOdometry, m_drive::followTrajectory, true, m_drive);
  private final Routines m_routines = new Routines(m_autoFactory);

  // Configure drive input stream
  SwerveInputStream driveInput =
      SwerveInputStream.of(
              m_drive.getSwerveDrive(),
              m_operatorController::getLeftY,
              m_operatorController::getLeftX)
          .withControllerRotationAxis(() -> -m_operatorController.getRightX())
          .deadband(0.1)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    // Set default drive command
    m_drive.setDefaultCommand(m_drive.driveFieldOriented(driveInput));

    m_autoChooser.addRoutine("Test Routine", m_routines::test);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putData("Xbox Controller Debug", m_operatorController.getHID());

    if (Robot.getInstance().isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
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
        .onTrue(Commands.runOnce(() -> m_elevator.setState(ElevatorSubsystem.ElevatorState.DOWN)));

    m_operatorController.rightBumper().onTrue(Commands.runOnce(m_coral::grab));
    m_operatorController.rightBumper().onFalse(Commands.runOnce(m_coral::idle));
    m_operatorController.leftBumper().onTrue(Commands.runOnce(m_coral::release));
    m_operatorController.leftBumper().onFalse(Commands.runOnce(m_coral::idle));
  }
}
