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
import frc.robot.commands.GrabCoral;
import frc.robot.commands.drive.Drive;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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
  private final FilteredJoystick m_leftJoystick =
      new FilteredJoystick(IOConstants.kLeftJoystickPort);
  private final FilteredJoystick m_rightJoystick =
      new FilteredJoystick(IOConstants.kRightJoystickPort);

  // Operator controller
  private final CommandXboxController m_controller =
      new CommandXboxController(IOConstants.kControllerPort);

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
    m_autoChooser.addRoutine("Left Score India 4", m_routines::scoreIndia4);
    m_autoChooser.addRoutine("Left Score India 4 Kilo 4", m_routines::scoreIndia4Kilo4);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Drive with controller
    // m_drive.setDefaultCommand(
    //     new Drive(
    //         m_drive,
    //         m_controller::getLeftY,
    //         m_controller::getLeftX,
    //         () -> -m_controller.getRightX(),
    //         () -> m_controller.getRightY(),
    //         () -> m_controller.leftBumper().getAsBoolean(),
    //         () -> m_controller.rightBumper().getAsBoolean(),
    //         Optional.empty()));

    // Drive with joysticks
    m_drive.setDefaultCommand(
        new Drive(
            m_drive,
            m_rightJoystick::getY,
            () -> -m_rightJoystick.getX(),
            () -> -m_leftJoystick.getX(),
            () -> m_leftJoystick.getY(),
            () -> m_leftJoystick.getButtonThree().getAsBoolean(),
            () -> m_rightJoystick.getButtonTwo().getAsBoolean(),
            Optional.of(m_rightJoystick::getThrottle)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVER JOYSTICKS
    {
      // Left joystick intakes coral, right joystick aligns to reef.  When both are held at once,
      // coral is released.
      // m_rightJoystick.getTriggerActive().whileTrue(); // Align to reef
      m_leftJoystick
          .getTriggerActive()
          .and(m_rightJoystick.getTriggerActive())
          .whileTrue(Commands.runOnce(m_coral::release));
      m_leftJoystick
          .getTriggerActive()
          .and(m_rightJoystick.getTriggerActive().negate())
          .whileTrue(new GrabCoral(m_coral));

      // Climber controls
      m_leftJoystick
          .getButtonEleven()
          .onTrue(Commands.runOnce(m_climber::up))
          .onFalse(Commands.runOnce(m_climber::inactive));
      m_leftJoystick
          .getButtonTen()
          .onTrue(Commands.runOnce(m_climber::down))
          .onFalse(Commands.runOnce(m_climber::inactive));

      // Reset gyro and set swerve drive to X-mode
      m_rightJoystick.getButtonThree().whileTrue(Commands.runOnce(m_drive::zeroGyro));
      m_rightJoystick.getButtonFour().whileTrue(Commands.runOnce(m_drive::lock));

      // Zero lift
      m_rightJoystick.getButtonEleven().onTrue(Commands.runOnce(m_elevator::zero));

      // Custom elevator
      m_leftJoystick.getButtonSix().whileTrue(Commands.runOnce(m_elevator::customUp));
      m_leftJoystick.getButtonSeven().whileTrue(Commands.runOnce(m_elevator::customDown));
    }

    // BUTTON BOARD
    {
      // Elevator controls

    }

    // CONTROLLER
    {
      // Climber controls
      m_controller
          .povUp()
          .onTrue(Commands.runOnce(m_climber::up))
          .onFalse(Commands.runOnce(m_climber::inactive));
      m_controller
          .povDown()
          .onTrue(Commands.runOnce(m_climber::downSlow))
          .onFalse(Commands.runOnce(m_climber::inactive));

      // Coral controls
      //   m_controller.leftBumper().whileTrue(new GrabCoral(m_coral));
      m_controller
          .leftTrigger()
          .onTrue(Commands.runOnce(m_coral::release))
          .onFalse(Commands.runOnce(m_coral::inactive));
      m_controller
          .rightTrigger()
          .onTrue(Commands.runOnce(m_coral::reverse))
          .onFalse(Commands.runOnce(m_coral::inactive));

      // Elevator Controls
      m_controller.a().onTrue(Commands.runOnce(m_elevator::L1));
      m_controller.x().onTrue(Commands.runOnce(m_elevator::L2));
      m_controller.b().onTrue(Commands.runOnce(m_elevator::L3));
      m_controller.y().onTrue(Commands.runOnce(m_elevator::L4));

      // Chute drop
      m_controller
          .leftStick()
          .and(m_controller.rightStick())
          .onTrue(Commands.runOnce(m_chute::drop));
    }
  }

  public void ResetStates() {
    Commands.runOnce(m_climber::inactive);
    Commands.runOnce(m_elevator::disable);
    Commands.runOnce(m_coral::inactive);
  }
}
