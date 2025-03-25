package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Handler.Coral;
import frc.robot.Robot;
import java.util.Optional;

public class CoralHandlerSubsystem extends SubsystemBase {
  // two neo 550s, 9:1 gearing on both, going in opposite directions.
  // looking on from the top, the right one going clockwise is intake, and
  // counterclockwise is
  // outtake.
  // vice versa for the other

  public enum CoralHandlerState {
    INACTIVE,
    GRAB,
    RELEASE,
    REVERSE,
    CUSTOM
  }

  private CoralHandlerState m_state = CoralHandlerState.INACTIVE;
  protected boolean m_hasCoral = false;
  private Optional<Double> m_customSpeed = Optional.empty();
  protected double m_speed;

  public CoralHandlerSubsystem() {
    Coral.kLeftSparkMax.configure(
        Coral.kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Coral.kRightSparkMax.configure(
        Coral.kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setState(m_state);
  }

  @Override
  public void periodic() {

    if (Robot.isReal()) {
      // update m_hasCoral
    }

    setSpeed(m_speed);

    log();
  }

  public void log() {
    SmartDashboard.putBoolean("Coral Handler Has Coral", m_hasCoral);
    SmartDashboard.putNumber("Coral Handler Speed", m_speed);
  }

  public void setSpeed(double speed) {

    Coral.kLeftSparkMax.set(-speed);
    Coral.kRightSparkMax.set(-speed);
  }

  /**
   * Set the state of the coral handler
   *
   * @param state {@link CoralHandlerState}
   */
  private void setState(CoralHandlerState state) {

    if (state == CoralHandlerState.CUSTOM && m_customSpeed.isEmpty()) {
      return;
    }

    m_state = state;
    m_speed = m_state == CoralHandlerState.CUSTOM ? m_customSpeed.get() : Coral.kSpeeds.get(state);
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
   * Set the custom speed, note that this will not change the state of the coral handler To do so,
   * call {@link #setState(CoralHandlerState.CUSTOM)}
   *
   * @param target double
   */
  public void setCustomSpeed(double target) {
    m_state = CoralHandlerState.CUSTOM;
    m_customSpeed = Optional.of(target);
  }

  /**
   * Get the speed of the coral handler
   *
   * @return double
   */
  public double getSpeed() {
    return m_speed;
  }

  /** Set state to index */
  public void grab() {
    setState(CoralHandlerState.GRAB);
  }

  /** Set state to score */
  public void release() {
    setState(CoralHandlerState.RELEASE);
  }

  /** Set state to inactive */
  public void inactive() {
    setState(CoralHandlerState.INACTIVE);
  }

  /** Set state to reverse */
  public void reverse() {
    setState(CoralHandlerState.REVERSE);
  }

  /**
   * Check if the coral handler has a coral
   *
   * @return boolean
   */
  public boolean hasCoral() {
    return m_hasCoral;
  }
}
