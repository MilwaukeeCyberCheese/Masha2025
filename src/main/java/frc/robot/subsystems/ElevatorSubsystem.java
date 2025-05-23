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
    CUSTOM
  }

  private ElevatorState m_state = ElevatorState.DISABLED;
  private Optional<Double> m_customHeight = Optional.empty();
  protected double m_height;

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
    log();

    // System.out.println(Elevator.kRightElevatorSparkMax.configAccessor.closedLoop.getP());

    Elevator.kElevatorController.setReference(m_height, ControlType.kPosition);
  }

  public void log() {
    // Log sensor data, etc. here
    SmartDashboard.putNumber("Elevator Height", m_height);
  }

  // TODO: add limits logic
  /**
   * Set the elevator to a specific state
   *
   * @param state {@link ElevatorState}
   */
  protected void setState(ElevatorState state) {

    if (state == ElevatorState.CUSTOM && m_customHeight.isEmpty()) {
      return;
    }

    m_state = state;

    // TODO: check that this overrides the PID
    if (m_state == ElevatorState.DISABLED) {
      Elevator.kLeftElevatorSparkMax.set(0);
      Elevator.kRightElevatorSparkMax.set(0);
      return;
    }

    m_height = state == ElevatorState.CUSTOM ? m_customHeight.get() : Elevator.kHeights.get(state);
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
   * Set the custom target height, this will also change the state of the elevator
   *
   * @param target double
   */
  public void setCustomTarget(double target) {
    m_customHeight = Optional.of(target);
    setState(ElevatorState.CUSTOM);
  }

  /**
   * Get the height of the elevator
   *
   * @return double
   */
  public double getHeight() {
    return m_height;
  }

  /**
   * Check if the elevator is at the desired height
   *
   * @return boolean
   */
  public boolean atHeight() {
    return Math.abs(m_height - Elevator.kLeftElevatorSparkMax.getAbsoluteEncoder().getPosition())
        < Elevator.kElevatorTolerance;
  }

  /** Set elevator state to disabled */
  public void disable() {
    setState(ElevatorState.DISABLED);
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
    Elevator.kRightElevatorSparkMax.getEncoder().setPosition(0);
  }

  public boolean atBottom() {
    // return Elevator.kElevatorLimitSwitch.isPressed();
    return false;
  }
}
