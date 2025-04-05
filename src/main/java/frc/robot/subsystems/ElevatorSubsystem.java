package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.utils.Stopwatch;

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
  protected double m_height;
  private Stopwatch m_zeroDebouncer = new Stopwatch();

  /** Creates a new ElevatorSubsystem. */
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

    m_zeroDebouncer.reset();
    m_zeroDebouncer.start();
  }

  @Override
  public void periodic() {
    log();

    // Re-zero the elevator when it's down, and debounce it so it doesn't occur more than once per
    // second
    if (atBottom() && m_zeroDebouncer.getTime() > 1000) zero();

    // Set the elevator to the desired height
    Elevator.kElevatorController.setReference(
        m_height, ControlType.kPosition, ClosedLoopSlot.kSlot0, Elevator.kG);
  }

  public void log() {
    // Log sensor data, etc. here
    SmartDashboard.putNumber("Expected Elevator Height", m_height);
    SmartDashboard.putNumber(
        "Elevator Height", Elevator.kRightElevatorSparkMax.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Speed", Elevator.kRightElevatorSparkMax.get());
    SmartDashboard.putString("Elevator State", m_state.toString());
    SmartDashboard.putBoolean("Elevator at Bottom", atBottom());
  }

  // TODO: add limits logic
  /**
   * Set the elevator to a specific state
   *
   * @param state {@link ElevatorState}
   */
  protected void setState(ElevatorState state) {

    m_state = state;

    if (state == ElevatorState.CUSTOM) return;

    // TODO: check that this overrides the PID
    if (m_state == ElevatorState.DISABLED) {
      Elevator.kLeftElevatorSparkMax.set(0);
      Elevator.kRightElevatorSparkMax.set(0);
      m_height = 0;
      return;
    }

    m_height = Elevator.kHeights.get(state);
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
    return Math.abs(m_height - Elevator.kRightElevatorSparkMax.getEncoder().getPosition())
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

  public void customUp() {
    m_height += Elevator.kCustomStep;
    setState(ElevatorState.CUSTOM);
  }

  public void customDown() {
    m_height -= Elevator.kCustomStep;
    setState(ElevatorState.CUSTOM);
  }

  /**
   * Adjust the elevator height by a custom increment
   *
   * @param increment
   */
  public void customAdjust(double increment) {
    m_height += increment;
    setState(ElevatorState.CUSTOM);
  }

  /** Zero the elevator encoder */
  public void zero() {
    Elevator.kRightElevatorSparkMax.getEncoder().setPosition(0);
    m_height = 0;
    setState(ElevatorState.DOWN);
    m_zeroDebouncer.reset();
    m_zeroDebouncer.start();
  }

  public boolean atBottom() {
    return !Elevator.kElevatorLimitSwitch.get();
  }
}
