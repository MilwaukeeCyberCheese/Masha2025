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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ChuteDrop;
import frc.robot.commands.GrabCoral;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.elevator.ManualElevatorPosition;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.sim.CoralHandlerSubsystemSim;
import frc.robot.subsystems.sim.ElevatorSubsystemSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.FilteredButton;
import java.io.File;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // All da various subsystems
  public final SwerveSubsystem m_drive =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
  public final ElevatorSubsystem m_elevator =
      Robot.isReal() ? new ElevatorSubsystem() : new ElevatorSubsystemSim();
  private final CoralHandlerSubsystem m_coral =
      Robot.isReal()
          ? new CoralHandlerSubsystem()
          : new CoralHandlerSubsystemSim(m_drive.getSimDrive(), m_elevator);
  private final ChuteSubsystem m_chute = new ChuteSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // Driver joysticks
  private final CommandXboxController m_driverController =
      new CommandXboxController(IOConstants.kDriverControllerPort);
  // Operator controller
  private final CommandXboxController m_operatorController =
      new CommandXboxController(IOConstants.kOperatorControllerPort);

  // Button Board
  private final FilteredButton m_buttons = new FilteredButton(IOConstants.kButtonBoardPort);

  // More auto stuff
  public final AutoChooser m_autoChooser = new AutoChooser();
  private final AutoFactory m_autoFactory =
      new AutoFactory(
          m_drive::getPose, m_drive::resetOdometry, m_drive::followTrajectory, true, m_drive);
  private final Routines m_routines = new Routines(m_autoFactory, m_elevator, m_coral);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    // Add routines to auto chooser
    m_autoChooser.addRoutine("Drive Out", m_routines::driveOut);
    m_autoChooser.addRoutine("Left Score India 4", m_routines::leftIndia4);
    m_autoChooser.addRoutine("Left Score India 4 Kilo 4", m_routines::leftIndia4Kilo4);
    m_autoChooser.addRoutine("Right Score Foxtrot 4", m_routines::rightFoxtrot4);
    m_autoChooser.addRoutine("Right Score Foxtrot 4 Delta 4", m_routines::rightFoxtrot4Delta4);
    m_autoChooser.addRoutine("Middle Own India 4", m_routines::middleOwnIndia4);
    m_autoChooser.addRoutine("Middle Opposing Foxtrot 4", m_routines::middleOpposingFoxtrot4);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Drive with controller
    m_drive.setDefaultCommand(
        new Drive(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(),
            () -> !m_driverController.rightBumper().getAsBoolean(),
            () -> m_driverController.leftBumper().getAsBoolean()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVER CONTROLLER
    {
      m_driverController
          .rightTrigger()
          .whileTrue(Commands.runOnce(m_coral::release))
          .onFalse(Commands.runOnce(m_coral::inactive));
      m_driverController.leftTrigger().whileTrue(new GrabCoral(m_coral));

      // Climber controls
      m_driverController
          .povUp()
          .onTrue(Commands.runOnce(m_climber::up))
          .onFalse(Commands.runOnce(m_climber::inactive));

      m_driverController
          .povDown()
          .onTrue(Commands.runOnce(m_climber::downSlow))
          .onFalse(Commands.runOnce(m_climber::inactive));

      // Reset gyro and set swerve drive to X-mode
      m_driverController.povLeft().onTrue(Commands.runOnce(m_drive::zeroGyro));
      m_driverController.povRight().whileTrue(Commands.runOnce(m_drive::lock));
    }

    // BUTTON BOARD
    {
      new Trigger(() -> m_buttons.getSwitch3()).onTrue(new ChuteDrop(m_chute, m_climber));
    }

    // OPERATOR CONTROLLER
    {
      // Climber controls
      m_operatorController
          .povUp()
          .onTrue(Commands.runOnce(m_climber::up))
          .onFalse(Commands.runOnce(m_climber::inactive));
      m_operatorController
          .povDown()
          .onTrue(Commands.runOnce(m_climber::downSlow))
          .onFalse(Commands.runOnce(m_climber::inactive));

      m_operatorController.povRight().onTrue(Commands.runOnce(m_elevator::zero));

      // Coral controls
      m_operatorController.leftBumper().whileTrue(new GrabCoral(m_coral));
      m_operatorController
          .leftTrigger()
          .onTrue(Commands.runOnce(m_coral::release))
          .onFalse(Commands.runOnce(m_coral::inactive));
      m_operatorController
          .rightTrigger()
          .onTrue(Commands.runOnce(m_coral::reverse))
          .onFalse(Commands.runOnce(m_coral::inactive));

      // Elevator Controls
      m_operatorController.a().onTrue(Commands.runOnce(m_elevator::L1));
      m_operatorController.x().onTrue(Commands.runOnce(m_elevator::L2));
      m_operatorController.b().onTrue(Commands.runOnce(m_elevator::L3));
      m_operatorController.y().onTrue(Commands.runOnce(m_elevator::L4));

      // Chute drop
      m_operatorController
          .leftStick()
          .and(m_operatorController.rightStick())
          .onTrue(new ChuteDrop(m_chute, m_climber));

      m_operatorController
          .rightBumper()
          .whileTrue(new ManualElevatorPosition(m_elevator, m_operatorController::getRightY));
    }
  }

  public void resetStates() {
    m_elevator.L1();
    m_climber.inactive();
    m_coral.inactive();
  }
}
