package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

// TODO: add sim support
public class ElevatorSubsystem extends SubsystemBase {

  public static enum ElevatorState {
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
    // Initialize motors, sensors, etc. here
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
   * @param state a state from {@link}ElevatorState
   */
  public void setState(ElevatorState state) {
    m_state = state;
    setHeight(Elevator.kElevatorHeights.get(state));
  }

  public ElevatorState getState() {
    return m_state;
  }

  /**
   * Set the height of the elevator
   *
   * @param height
   */
  private void setHeight(double height) {
    Elevator.kElevatorController.setReference(height, ControlType.kMAXMotionPositionControl);
  }
}
