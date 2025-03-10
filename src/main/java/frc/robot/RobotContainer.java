// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.drive.AlignToAprilTag;
import frc.robot.commands.drive.Drive;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.sim.CoralHandlerSubsystemSim;
import frc.robot.subsystems.sim.ElevatorSubsystemSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.utils.FilteredButton;
import frc.robot.utils.FilteredJoystick;
import java.io.File;
import java.util.ArrayList;
import java.util.Objects;
import swervelib.SwerveInputStream;
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
  private final FilteredJoystick m_driverLeftJoystick =
      new FilteredJoystick(IOConstants.kLeftJoystickPort);
  private final FilteredJoystick m_driverRightJoystick =
      new FilteredJoystick(IOConstants.kRightJoystickPort);

  // Operator controller
  private final CommandXboxController m_operatorController =
      new CommandXboxController(IOConstants.kOperatorControllerPort);

  // Button Board
  private final FilteredButton m_buttonBoard = new FilteredButton(IOConstants.kButtonBoardPort);

  public final AutoChooser m_autoChooser = new AutoChooser();
  private final AutoFactory m_autoFactory =
      new AutoFactory(
          m_drive::getPose, m_drive::resetOdometry, m_drive::followTrajectory, true, m_drive, this::logTrajectory);
  private final Routines m_routines = new Routines(m_autoFactory);

  private final FieldObject2d allPositions = this.m_drive.getSwerveDrive().field.getObject("Positions");

  private String lastTrajectory;
  private final FieldObject2d autoTrajectoryObj = this.m_drive.getSwerveDrive().field.getObject("Auto Trajectory");
  private final FieldObject2d allTrajectoriesObj = this.m_drive.getSwerveDrive().field.getObject("All Trajectories");

  // Configure drive input stream
  SwerveInputStream driveInput;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    // Set default drive command
    m_drive.setDefaultCommand(m_drive.driveFieldOriented(driveInput));

    m_autoChooser.addRoutine("Test Routine", m_routines::test);
    m_autoChooser.addRoutine("Blue Processor Routine", m_routines::blueProcessor);
    m_autoChooser.addRoutine("Blue Coral Station Routine", m_routines::blueCoralStation);
    m_autoChooser.addRoutine("Blue Reef K Routine", m_routines::blueCoralToReefK);
    m_autoChooser.addRoutine("Blue Test Full Routine", m_routines::blueTestFull);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putData("Xbox Controller Debug", m_operatorController.getHID());

    if (Robot.getInstance().isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    if (IOConstants.kTestMode) {
      m_drive.setDefaultCommand(
          new Drive(
              m_drive,
              m_operatorController::getRightX,
              m_operatorController::getLeftY,
              () -> -m_operatorController.getRightX(),
              () -> m_operatorController.rightBumper().getAsBoolean(),
              Optional.empty()));
    } else {
      m_drive.setDefaultCommand(
          new Drive(
              m_drive,
              m_driverLeftJoystick::getX,
              m_driverLeftJoystick::getY,
              m_driverRightJoystick::getX,
              m_driverRightJoystick::getButtonTwo,
              Optional.of(m_driverLeftJoystick::getThrottle)));
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
    if (IOConstants.kTestMode && false) {

    } else {

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

      m_operatorController.rightStick()
              .onTrue(new AlignToAprilTag(this.m_drive, () -> 18, () -> Vision.Camera.CENTER_CAM));
    }
  }

  public void clearPositionDebug() {
    this.allPositions.setPoses();
  }

  public void updatePositionDebug() {
    final var newPoses = this.allPositions.getPoses();
    final var currentPose = this.m_drive.getPose();

    if (!newPoses.isEmpty() && newPoses.getLast().getTranslation().getDistance(currentPose.getTranslation()) >= 4) newPoses.clear();

    newPoses.add(currentPose);
    this.allPositions.setPoses(newPoses);
  }

  public void clearAutoTrajectories() {
    this.lastTrajectory = null;
    this.autoTrajectoryObj.setPoses();
    this.allTrajectoriesObj.setPoses();
  }

  private void logTrajectory(Trajectory<SwerveSample> trajectory, boolean isStart) {
    if (isStart) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) trajectory = trajectory.flipped();

      final var poses = new ArrayList<Pose2d>(trajectory.samples().size());
      for (final var swerveSample : trajectory.samples()) {
        poses.add(swerveSample.getPose());
      }
      this.lastTrajectory = trajectory.name();
      this.autoTrajectoryObj.setPoses(poses);
      final var oldAllPoses = this.allTrajectoriesObj.getPoses();
      oldAllPoses.addAll(poses);
      this.allTrajectoriesObj.setPoses(oldAllPoses);
    } else if (Objects.equals(this.lastTrajectory, trajectory.name())) {
      this.autoTrajectoryObj.setPoses();
      this.lastTrajectory = null;
    }
  }
}
