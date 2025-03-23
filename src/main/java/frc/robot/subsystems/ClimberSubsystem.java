package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;
import java.util.Optional;

// TODO: add sim support
public class ClimberSubsystem extends SubsystemBase {

  public static enum ClimberState {
    INACTIVE,
    UP,
    DOWN,
    DOWNSLOW,
    CUSTOM
  }

  private ClimberState m_state = ClimberState.INACTIVE;
  private Optional<Double> m_customSpeed = Optional.empty();
  protected double m_speed;

  public ClimberSubsystem() {
    Climber.kClimberSparkMax.configure(
        Climber.kClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setState(m_state);
  }

  @Override
  public void periodic() {
    Climber.kClimberSparkMax.set(m_speed);

    log();
  }

  public void log() {
    SmartDashboard.putNumber(
        "Climber Position", Climber.kClimberSparkMax.getAbsoluteEncoder().getPosition());
  }

  // TODO: add logic for limits
  /**
   * Set the state of the climber
   *
   * @param state options from {@link ClimberState}
   */
  private void setState(ClimberState state) {
    if (state == ClimberState.CUSTOM && m_customSpeed.isEmpty()) {
      return;
    }

    m_state = state;

    m_speed = m_state == ClimberState.CUSTOM ? m_customSpeed.get() : Climber.kSpeeds.get(m_state);
  }

  /**
   * Get the current state of the climber
   *
   * @return {@link ClimberState}
   */
  public ClimberState getState() {
    return m_state;
  }

  /**
   * Get the current position of the climber
   *
   * @return double
   */
  public double getSpeed() {
    return m_speed;
  }

  /**
   * Get the position of the climber
   *
   * @return double
   */
  private double getPosition() {
    return Climber.kClimberSparkMax.getAbsoluteEncoder().getPosition();
  }

  /**
   * Get whether the climber is fully down
   *
   * @return boolean
   */
  public boolean isDown() {
    return getPosition() <= Climber.kDownPosition;
  }

  /**
   * Set the custom position of the climber Changes the state of the climber
   *
   * @param position
   */
  public void setCustomPosition(double position) {
    m_customSpeed = Optional.of(position);
    m_state = ClimberState.CUSTOM;
  }

  /** Set the climber to inactive */
  public void inactive() {
    setState(ClimberState.INACTIVE);
  }

  /** Set the climber to go up */
  public void up() {
    setState(ClimberState.UP);
  }

  /** Set the climber to go down */
  public void down() {
    setState(ClimberState.DOWN);
  }

  /** Set the climber to go down slow */
  public void downSlow() {
    setState(ClimberState.DOWNSLOW);
  }
}
