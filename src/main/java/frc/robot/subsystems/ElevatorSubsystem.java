package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

// TODO: add sim support
public class ElevatorSubsystem extends SubsystemBase {

  public static enum ElevatorState {
    DISABLED,
    DOWN,
    L1,
    L2,
    L3,
    L4,
    ALGAE_FROM_REEF,
    ALGAE_FROM_FLOOR // TBD if this is needed depending on how the intake for algae works
  }

  private ElevatorState m_state = ElevatorState.DOWN;

  public ElevatorSubsystem() {
    Elevator.kLeftElevatorSparkMax.configure(
        Elevator.kLeftElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    Elevator.kRightElevatorSparkMax.configure(
        Elevator.kRightElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    setState(m_state);
  }

  // Methods to set motor speeds, etc. go here

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    log();
  }

  public void log() {
    // Log sensor data, etc. here
  }

  // TODO: add limits logic
  /**
   * Set the elevator to a specific state
   *
   * @param state {@link ElevatorState}
   */
  public void setState(ElevatorState state) {
    m_state = state;

    // TODO: check that this overrides the PID
    if (m_state == ElevatorState.DISABLED) {
      Elevator.kLeftElevatorSparkMax.set(0);
      Elevator.kRightElevatorSparkMax.set(0);
      return;
    }

    setHeight(Elevator.kElevatorHeights.get(state));
  }

  /**
   * Check if the elevator is at the desired height
   *
   * @return boolean
   */
  public boolean atHeight() {
    return Math.abs(
            Elevator.kElevatorHeights.get(m_state)
                - Elevator.kLeftElevatorSparkMax.getAbsoluteEncoder().getPosition())
        < Elevator.kElevatorTolerance;
  }

  /**
   * Get the current state of the elevator
   *
   * @return {@link ElevatorState}
   */
  public ElevatorState getState() {
    return m_state;
  }

  /**
   * Set the height of the elevator
   *
   * @param height double
   */
  private void setHeight(double height) {
    Elevator.kElevatorController.setReference(height, ControlType.kMAXMotionPositionControl);
  }

  // TODO: test this
  /**
   * Zero the absolute encoder of the elevator
   *
   * <p>Should only be called when the elevator is at the bottom
   */
  public void zero() {
    setState(ElevatorState.DISABLED);

    // Zero the offset, as we don't know what the prior one was
    Elevator.kLeftElevatorConfig.absoluteEncoder.zeroOffset(0);
    Elevator.kLeftElevatorSparkMax.configure(
        Elevator.kLeftElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the offset to the current position
    Elevator.kLeftElevatorConfig.absoluteEncoder.zeroOffset(
        Elevator.kLeftElevatorSparkMax.getAbsoluteEncoder().getPosition());
    Elevator.kLeftElevatorSparkMax.configure(
        Elevator.kLeftElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    setState(ElevatorState.DOWN);
  }
}
