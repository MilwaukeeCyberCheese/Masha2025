// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.drive.Drive;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
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
  private final ChuteSubsystem m_chute = new ChuteSubsystem();
  private final AlgaeHandlerSubsystem m_algae = new AlgaeHandlerSubsystem();

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

  public final AutoChooser m_autoChooser = new AutoChooser();
  private final AutoFactory m_autoFactory =
      new AutoFactory(
          m_drive::getPose, m_drive::resetOdometry, m_drive::followTrajectory, true, m_drive);
  private final Routines m_routines = new Routines(m_autoFactory);

  private ControllerState controllerState = ControllerState.UNKNOWN;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.setControllerState(ControllerState.UNKNOWN);
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

    m_autoChooser.addRoutine("Test Routine", m_routines::test);
    m_autoChooser.addRoutine("Blue Processor Routine", m_routines::blueProcessor);
    m_autoChooser.addRoutine("Blue Coral Station Routine", m_routines::blueCoralStation);
    m_autoChooser.addRoutine("Blue Reef K Routine", m_routines::blueCoralToReefK);
    m_autoChooser.addRoutine("Blue Test Full Routine", m_routines::blueTestFull);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_controller.start().onTrue(Commands.runOnce(() -> this.controllerState = ControllerState.XBOX));
    m_leftJoystick.getButtonNine()
            .onTrue(Commands.runOnce(() -> this.controllerState = ControllerState.JOYSTICKS));
    m_leftJoystick.getButtonNine()
            .onTrue(Commands.runOnce(() -> this.controllerState = ControllerState.JOYSTICKS));

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
      return m_controller.getLeftX();
    } else {
      return m_leftJoystick.getX();
    }
  }

  public double getControllerY() {
    if (this.isXboxController()) {
      return m_controller.getLeftY();
    } else {
      return m_leftJoystick.getY();
    }
  }

  public double getContollerRotation() {
    if (this.isXboxController()) {
      return -m_controller.getRightX();
    } else {
      return m_rightJoystick.getX();
    }
  }

  public boolean getControllerSlow() {
    if (this.isXboxController()) {
      return m_controller.rightBumper().getAsBoolean();
    } else {
      return m_rightJoystick.getButtonTwo().getAsBoolean();
    }
  }

  public double getControllerThrottle() {
    if (this.isXboxController()) {
      return 1.0;
    } else {
      return m_leftJoystick.getThrottle();
    }
  }

  private boolean isAnyJoystickConnected() {
    return this.m_leftJoystick.getJoystick().isConnected() || this.m_rightJoystick.getJoystick().isConnected();
  }

  private void setControllerState(ControllerState newState) {
    this.controllerState = newState;
    System.out.println("Controller state now " + this.controllerState);
    SmartDashboard.putString("Controller State", this.controllerState.toString());
  }

  public void updateControllerConnections() {
    if (this.controllerState == ControllerState.XBOX && !this.m_controller.isConnected()) {
      this.setControllerState(ControllerState.UNKNOWN);
    } else if (this.controllerState == ControllerState.JOYSTICKS && !this.isAnyJoystickConnected()) {
      this.setControllerState(ControllerState.UNKNOWN);
    }

    if (this.controllerState == ControllerState.UNKNOWN) {
      if (this.m_controller.isConnected()) this.setControllerState(ControllerState.XBOX);
      else if (this.isAnyJoystickConnected()) this.setControllerState(ControllerState.JOYSTICKS);
    }
  }

  private enum ControllerState {
    UNKNOWN,
    XBOX,
    JOYSTICKS,
  }
}
