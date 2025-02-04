package frc.robot.subsystems;

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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

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

  public CoralHandlerSubsystem(AbstractDriveTrainSimulation driveSim) {
    m_drive = driveSim;
    Coral.m_left.configure(
        Coral.m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Coral.m_right.configure(
        Coral.m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Rectangle intake = new Rectangle(.762, .245);
    intake.translate(new Vector2(0, .762));
    m_intakeSim = new IntakeSimulation("Coral", driveSim, intake, 1);
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
    Coral.m_leftController.setReference(speed, ControlType.kMAXMotionVelocityControl);
    Coral.m_rightController.setReference(speed, ControlType.kMAXMotionVelocityControl);
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
        // TODO: make this dependent on elevator position
        SimulatedArena
            .getInstance()
            .addGamePieceProjectile(
              new ReefscapeCoralOnFly(
                m_drive.getSimulatedDriveTrainPose().getTranslation(),
                new Translation2d(0.35, 0), // mechanism position, this is where elevator comes into play i think
                m_drive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                m_drive.getSimulatedDriveTrainPose().getRotation(),
                Meters.of(1.28), // eject height
                MetersPerSecond.of(2), // eject speed
                Radians.of(0.610865238)
              )
            );
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

  /** Intake a simulated coral from thin air, like magic ✨ */
  public void getSimCoral() {
    Class<?> clazz = m_intakeSim.getClass();
    // we need to increment the private field, gamePiecesInIntakeCount
    // TODO: upstream this and make it not hacky - see Shenzhen-Robotics-Alliance/maple-sim#106
    try {
      java.lang.reflect.Field field = clazz.getDeclaredField("gamePiecesInIntakeCount");
      field.setAccessible(true);
      int value = (int) field.get(m_intakeSim);
      if (value < 1) {
        field.set(m_intakeSim, value + 1);
      } else {
        System.err.println("You already have a coral, you greedy bastard!");
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
