package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Handler.Coral;
import frc.robot.Constants.Sensors;
import frc.robot.Robot;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class CoralHandlerSubsystem extends SubsystemBase {
  // two neo 550s, 9:1 gearing on both, going in opposite directions.
  // looking on from the top, the right one going clockwise is intake, and
  // counterclockwise is
  // outtake.
  // vice versa for the other

  public final IntakeSimulation m_intakeSim;
  private final AbstractDriveTrainSimulation m_drive;

  public enum CoralHandlerState {
    kInactive,
    kGrab,
    kRelease,
  }

  private CoralHandlerState m_state = CoralHandlerState.kInactive;
  private boolean m_hasCoral = false;

  private final ElevatorSubsystem m_elevator;

  public CoralHandlerSubsystem(AbstractDriveTrainSimulation driveSim, ElevatorSubsystem elevator) {
    m_drive = driveSim;
    Coral.kLeftSparkMax.configure(
        Coral.kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Coral.kRightSparkMax.configure(
        Coral.kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Rectangle intake = new Rectangle(.762, .245);
    intake.translate(new Vector2(0, .762));
    m_intakeSim = new IntakeSimulation("Coral", driveSim, intake, 1);
    m_elevator = elevator;
  }

  /**
   * Get the current state of the coral handler
   *
   * @return {@link CoralHandlerState}
   */
  public CoralHandlerState getState() {
    return m_state;
  }

  /**
   * Check if the coral handler has a coral
   *
   * @return boolean
   */
  public boolean hasCoral() {
    return m_hasCoral;
  }

  /**
   * Set the state of the coral handler
   *
   * @param state {@link CoralHandlerState}
   */
  public void setState(CoralHandlerState state) {
    m_state = state;
    setSpeed(Coral.kSpeeds.get(state));
  }

  /**
   * Set the speed of the coral handler
   *
   * @param speed double
   */
  private void setSpeed(double speed) {
    Coral.kLeftController.setReference(speed, ControlType.kMAXMotionVelocityControl);
    Coral.kRightController.setReference(speed, ControlType.kMAXMotionVelocityControl);
  }

  /**
   * Get whether the coral handler is at the right speed
   *
   * @return boolean
   */
  public boolean atSpeed() {
    return Math.abs(Coral.kSpeeds.get(m_state) - Coral.kLeftSparkMax.getEncoder().getVelocity())
            < Coral.kTolerance
        && Math.abs(Coral.kSpeeds.get(m_state) - Coral.kRightSparkMax.getEncoder().getVelocity())
            < Coral.kTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Handler Has Coral", m_hasCoral);
    // TODO: think about if we want to do this logic in a command instead (we probably do)
    if (Robot.isReal()) {
      m_hasCoral = Sensors.handlerDistanceSensor.getRange(Unit.kInches) < 5;
    } else {
      if (m_state == CoralHandlerState.kGrab) {
        m_intakeSim.startIntake();
      } else {
        m_intakeSim.stopIntake();
      }
      if (m_state == CoralHandlerState.kRelease && m_intakeSim.obtainGamePieceFromIntake()) {
        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new ReefscapeCoralOnFly(
                    m_drive.getSimulatedDriveTrainPose().getTranslation(),
                    new Translation2d(Inches.of(16.0), Inches.of(0)),
                    m_drive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    m_drive.getSimulatedDriveTrainPose().getRotation(),
                    m_elevator.getSimEjectHeight(),
                    MetersPerSecond.of(0.8), // eject speed
                    Radians.of(-Math.PI / 9)));
      }
      m_hasCoral = m_intakeSim.getGamePiecesAmount() != 0;
    }

    log();
  }

  public void log() {
    // SmartDashboard.putBoolean("Coral Handler Has Coral", m_hasCoral);
  }

  /** Set state to index */
  public void grab() {
    setState(CoralHandlerState.kGrab);
  }

  /** Set state to score */
  public void release() {
    setState(CoralHandlerState.kRelease);
  }

  /** Set state to inactive */
  public void idle() {
    setState(CoralHandlerState.kInactive);
  }

  /** Intake a simulated coral from thin air, like magic */
  public void getSimCoral() {
    if (!m_intakeSim.addGamePieceToIntake()) {
      System.err.println("You already have a coral, you greedy bastard!");
    }
  }
}
