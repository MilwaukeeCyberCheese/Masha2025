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
    INVERSE,
    CUSTOM
  }

  private CoralHandlerState m_state = CoralHandlerState.INACTIVE;
  protected boolean m_hasCoral = false;
  private Optional<Double> m_customSpeed = Optional.empty();
  protected double m_leftSpeed;
  protected double m_rightSpeed;

  /** Creates a new CoralHandlerSubsystem. */
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

    setSpeed(m_leftSpeed, m_rightSpeed);

    log();
  }

  public void log() {
    SmartDashboard.putBoolean("Coral Handler Has Coral", m_hasCoral);
    SmartDashboard.putNumber("Coral Handler Left Speed", m_leftSpeed);
    SmartDashboard.putNumber("Coral Handler Right Speed", m_rightSpeed);
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {

    Coral.kLeftSparkMax.set(-leftSpeed);
    Coral.kRightSparkMax.set(-rightSpeed);
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
    m_leftSpeed = m_state == CoralHandlerState.CUSTOM ? m_customSpeed.get() : Coral.kSpeeds.get(state)[0];
    m_rightSpeed = m_state == CoralHandlerState.CUSTOM ? m_customSpeed.get() : Coral.kSpeeds.get(state)[1];
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
    return m_leftSpeed;
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

  public void inverse() {
    setState(CoralHandlerState.INVERSE);
  }

  /**
   * Check if the coral handler has a coral
   *
   * @return boolean
   */
  public boolean hasCoral() {
    return !Coral.kBeamBreak.get();
  }
}
