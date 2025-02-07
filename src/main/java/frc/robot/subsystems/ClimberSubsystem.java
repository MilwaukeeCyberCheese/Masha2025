package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;
import java.util.Optional;

// TODO: add sim support
public class ClimberSubsystem extends SubsystemBase {

  public static enum ClimberState {
    WAITING,
    STOWED,
    CLIMB,
    CUSTOM
  }

  private ClimberState m_state = ClimberState.STOWED;
  private double m_position = 0;
  private Optional<Double> m_customPosition = Optional.empty();

  public ClimberSubsystem() {
    Climber.kClimberSparkMax.configure(
        Climber.kClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setState(m_state);
  }

  // TODO: add logic for limits
  /**
   * Set the state of the climber
   *
   * @param state options from {@link ClimberState}
   */
  public void setState(ClimberState state) {
    if (state == ClimberState.CUSTOM && m_customPosition.isEmpty()) {
      return;
    }

    m_state = state;

    m_position =
        m_state == ClimberState.CUSTOM ? m_customPosition.get() : Climber.kPositions.get(m_state);
  }

  /**
   * Set the custom position of the climber
   *
   * @param position
   */
  public void setCustomPosition(double position) {
    m_customPosition = Optional.of(position);
  }

  /**
   * Check if the climber is at the desired position
   *
   * @return boolean
   */
  public boolean atPosition() {
    return Math.abs(
            Climber.kPositions.get(m_state)
                - Climber.kClimberSparkMax.getAbsoluteEncoder().getPosition())
        < Climber.kClimberTolerance;
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
  public double getPosition() {
    return Climber.kClimberSparkMax.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Climber.kClimberController.setReference(m_position, ControlType.kPosition);

    log();
  }

  // TODO; add logging code
  public void log() {
    SmartDashboard.putNumber(
        "Climber Position", Climber.kClimberSparkMax.getAbsoluteEncoder().getPosition());
  }
}
