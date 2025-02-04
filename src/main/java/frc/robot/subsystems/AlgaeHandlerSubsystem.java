package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Handler.Algae;

// TODO: add sim support
public class AlgaeHandlerSubsystem extends SubsystemBase {

  public static enum AlgaeHandlerPositionState {
    STOWED,
    GRAB_FROM_REEF,
    GRAB_FROM_GROUND
  }

  public static enum AlgaeHandlerIntakeState {
    INTAKE,
    OUTTAKE,
    STOPPED
  }

  private AlgaeHandlerPositionState m_positionState = AlgaeHandlerPositionState.STOWED;
  private AlgaeHandlerIntakeState m_intakeState = AlgaeHandlerIntakeState.STOPPED;

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
    m_positionState = state;
    setPosition(Algae.kPositionStates.get(state));
  }

  /**
   * Set the position of the algae handler
   *
   * @param position
   */
  private void setPosition(double position) {
    Algae.kPositionController.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Check if the algae handler is at the desired position
   *
   * @return boolean
   */
  public boolean atState() {
    return Math.abs(
            Algae.kPositionStates.get(m_positionState)
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
   * Set the state of the algae handler's intake
   *
   * @param state options from {@link AlgaeHandlerIntakeState}
   */
  public void setIntakeState(AlgaeHandlerIntakeState state) {
    m_intakeState = state;
    setIntake(Algae.kIntakeStates.get(state));
  }

  /**
   * Set the speed of the algae handler's intake
   *
   * @param speed
   */
  private void setIntake(double speed) {
    Algae.kIntakeController.setReference(speed, ControlType.kMAXMotionVelocityControl);
  }

  /**
   * Get whether the algae handler's intake is at the desired state
   *
   * @return boolean
   */
  public boolean atIntakeState() {
    return Math.abs(
            Algae.kIntakeStates.get(m_intakeState)
                - Algae.kIntakeSparkMax.getEncoder().getVelocity())
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    log();
  }

  // TODO; add logging code
  public void log() {}
}
