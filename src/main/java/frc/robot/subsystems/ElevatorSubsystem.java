package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import java.util.Optional;

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
    ALGAE_FROM_FLOOR, // TBD if this is needed depending on how the intake for algae works
    CUSTOM
  }

  private ElevatorState state = ElevatorState.DOWN;
  private Optional<Double> customHeight = null;
  protected double height;

  public ElevatorSubsystem() {
    Elevator.kLeftElevatorSparkMax.configure(
        Elevator.LEFT_ELEVATOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    Elevator.RIGHT_ELEVATOR_SPARK_MAX.configure(
        Elevator.RIGHT_ELEVATOR_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    setState(state);
  }

  // Methods to set motor speeds, etc. go here

  @Override
  public void periodic() {
    log();

    Elevator.ELEVATOR_CONTROLLER.setReference(height, ControlType.kMAXMotionPositionControl);
  }

  public void log() {
    // Log sensor data, etc. here
    SmartDashboard.putNumber("Elevator Height", height);
    SmartDashboard.putString("Elevator State", state.toString());
  }

  // TODO: add limits logic
  /**
   * Set the elevator to a specific state
   *
   * @param state {@link ElevatorState}
   */
  public void setState(ElevatorState state) {

    if (state == ElevatorState.CUSTOM && customHeight.isEmpty()) {
      return;
    }

    this.state = state;

    // TODO: check that this overrides the PID
    if (this.state == ElevatorState.DISABLED) {
      Elevator.kLeftElevatorSparkMax.set(0);
      Elevator.RIGHT_ELEVATOR_SPARK_MAX.set(0);
      return;
    }

    height = state == ElevatorState.CUSTOM ? customHeight.get() : Elevator.HEIGHTS.get(state);
  }

  /**
   * Get the current state of the elevator
   *
   * @return {@link ElevatorState}
   */
  public ElevatorState getState() {
    return state;
  }

  /**
   * Set the custom target height, note that this will not change the state of the elevator To do
   * so, call {@link #setState(ElevatorState.CUSTOM)}
   *
   * @param target double
   */
  public void setCustomTarget(double target) {
    state = ElevatorState.CUSTOM;
    customHeight = Optional.of(target);
  }

  /**
   * Get the height of the elevator
   *
   * @return double
   */
  public double getHeight() {
    return height;
  }

  /**
   * Check if the elevator is at the desired height
   *
   * @return boolean
   */
  public boolean atHeight() {
    return Math.abs(height - Elevator.kLeftElevatorSparkMax.getAbsoluteEncoder().getPosition())
        < Elevator.ELEVATOR_TOLERANCE;
  }

  /** Set elevator state to down */
  public void down() {
    setState(ElevatorState.DOWN);
  }

  /** Set elevator state to L1 */
  public void L1() {
    setState(ElevatorState.L1);
  }

  /** Set elevator state to L2 */
  public void L2() {
    setState(ElevatorState.L2);
  }

  /** Set elevator state to L3 */
  public void L3() {
    setState(ElevatorState.L3);
  }

  /** Set elevator state to L4 */
  public void L4() {
    setState(ElevatorState.L4);
  }

  // TODO: test this
  /**
   * Zero the absolute encoder of the elevator
   *
   * <p>Should only be called when the elevator is at the bottom
   *
   * @param persistMode {@link PersistMode} only call this when intending to save the new offset,
   *     note that this will cause the spark to become unresponsive for a short period of time
   */
  public void zero() {
    Elevator.kLeftElevatorSparkMax.getEncoder().setPosition(0);
  }

  public boolean atBottom() {
    return Elevator.ELEVATOR_LIMIT_SWITCH.isPressed();
  }
}
