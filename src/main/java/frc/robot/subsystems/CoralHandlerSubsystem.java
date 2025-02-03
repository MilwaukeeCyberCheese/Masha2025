package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Handler.Coral;
import frc.robot.Constants.Sensors;
import frc.robot.Robot;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CoralHandlerSubsystem extends SubsystemBase {
  // two neo 550s, 9:1 gearing on both, going in opposite directions.
  // looking on from the top, the right one going clockwise is intake, and
  // counterclockwise is
  // outtake.
  // vice versa for the other

  public final IntakeSimulation m_intakeSim;

  public enum CoralHandlerState {
    kInactive,
    kIndex,
    kScore,
  }

  private CoralHandlerState m_state = CoralHandlerState.kInactive;
  private boolean m_hasCoral = false;

  public CoralHandlerSubsystem(AbstractDriveTrainSimulation driveSim) {

    Coral.m_left.configure(
        Coral.m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Coral.m_right.configure(
        Coral.m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_intakeSim = new IntakeSimulation("Coral", driveSim, new Rectangle(.762, 1.007), 1);
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
    Robot.getInstance();
    // TODO: think about if we want to do this logic in a command instead (we probably do)
    if (Robot.isReal()) {
      m_hasCoral = Sensors.handlerDistanceSensor.getRange(Unit.kInches) < 5;
    } else {
      if (m_state == CoralHandlerState.kIndex) {
        m_intakeSim.startIntake();
      } else {
        m_intakeSim.stopIntake();
      }
      m_hasCoral = m_intakeSim.getGamePiecesAmount() != 0;
    }

    log();
  }

  public void log() {
    // SmartDashboard.putBoolean("Coral Handler Has Coral", m_hasCoral);
  }

  /** Set state to index */
  public void index() {
    setState(CoralHandlerState.kIndex);
  }

  /** Set state to score */
  public void score() {
    setState(CoralHandlerState.kScore);
  }
}
