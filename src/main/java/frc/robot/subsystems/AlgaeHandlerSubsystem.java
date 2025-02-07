package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Handler.Algae;
import java.util.Optional;

// TODO: add sim support
public class AlgaeHandlerSubsystem extends SubsystemBase {

  public static enum AlgaeHandlerPositionState {
    STOWED,
    GRAB_FROM_REEF,
    GRAB_FROM_GROUND,
    CUSTOM
  }

  public static enum AlgaeHandlerIntakeState {
    INTAKE,
    OUTTAKE,
    STOPPED,
    CUSTOM
  }

  private AlgaeHandlerPositionState m_positionState = AlgaeHandlerPositionState.STOWED;
  private AlgaeHandlerIntakeState m_intakeState = AlgaeHandlerIntakeState.STOPPED;

  private double m_position;
  private double m_speed;

  private Optional<Double> m_customPosition = Optional.empty();
  private Optional<Double> m_customSpeed = Optional.empty();

  public AlgaeHandlerSubsystem() {
    Algae.kPositionSparkMax.configure(
        Algae.kPositionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Algae.kIntakeSparkMax.configure(
        Algae.kIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setPositionState(AlgaeHandlerPositionState.STOWED);
    setIntakeState(AlgaeHandlerIntakeState.STOPPED);
  }

  // TODO: add logic for limits
  /**
   * Set the state of the algae handler's position
   *
   * @param state options from {@link AlgaeHandlerPositionState}
   */
  public void setPositionState(AlgaeHandlerPositionState state) {
    if (state == AlgaeHandlerPositionState.CUSTOM && m_customPosition.isEmpty()) {
      return;
    }

    m_positionState = state;

    m_position =
        m_positionState == AlgaeHandlerPositionState.CUSTOM
            ? m_customPosition.get()
            : Algae.kPositions.get(state);
  }

  /**
   * Set custom position for the algae handler This does not change the state of the algae handler
   *
   * @param position double
   */
  public void setCustomPosition(double position) {
    m_customPosition = Optional.of(position);
  }

  /**
   * Check if the algae handler is at the desired position
   *
   * @return boolean
   */
  public boolean atState() {
    return Math.abs(
            Algae.kPositions.get(m_positionState)
                - Algae.kPositionSparkMax.getAbsoluteEncoder().getPosition())
        < Algae.kPositionTolerance;
  }

  /**
   * Get the current state of the algae handler's position
   *
   * @return {@link AlgaeHandlerPositionState}
   */
  public AlgaeHandlerPositionState getState() {
    return m_positionState;
  }

  /**
   * Get the position of the algae handler
   *
   * @return double
   */
  public double getPosition() {
    return m_position;
  }

  /**
   * Set the state of the algae handler's intake
   *
   * @param state options from {@link AlgaeHandlerIntakeState}
   */
  public void setIntakeState(AlgaeHandlerIntakeState state) {
    if (state == AlgaeHandlerIntakeState.CUSTOM && m_customSpeed.isEmpty()) {
      return;
    }

    m_intakeState = state;

    m_speed =
        m_intakeState == AlgaeHandlerIntakeState.CUSTOM
            ? m_customSpeed.get()
            : Algae.kSpeeds.get(state);
  }

  /**
   * Set custom speed for the algae handler's intake
   *
   * @param speed double
   */
  public void setCustomSpeed(double speed) {
    m_customSpeed = Optional.of(speed);
  }

  /**
   * Get whether the algae handler's intake is at the desired state
   *
   * @return boolean
   */
  public boolean atIntakeSpeed() {
    return Math.abs(
            Algae.kSpeeds.get(m_intakeState) - Algae.kIntakeSparkMax.getEncoder().getVelocity())
        < Algae.kIntakeTolerance;
  }

  /**
   * Get the current state of the algae handler's intake
   *
   * @return {@link AlgaeHandlerIntakeState}
   */
  public AlgaeHandlerIntakeState getIntakeState() {
    return m_intakeState;
  }

  /**
   * Get the speed of the algae handler's intake
   *
   * @return double
   */
  public double getIntakeSpeed() {
    return m_speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Algae.kPositionController.setReference(m_position, ControlType.kPosition);
    Algae.kIntakeController.setReference(m_speed, ControlType.kVelocity);

    log();
  }

  // TODO; add logging code
  public void log() {
    SmartDashboard.putNumber(
        "Algae Handler Position", Algae.kPositionSparkMax.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber(
        "Algae Handler Intake Speed", Algae.kIntakeSparkMax.getEncoder().getVelocity());
  }
}
