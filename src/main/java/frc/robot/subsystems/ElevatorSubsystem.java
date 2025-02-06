package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
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

  private double m_simHeight = 0.0;
  private double m_simTargetHeight = 0.0;

  StructArrayPublisher<Pose3d> m_simPoseArray =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Simulation/ElevatorPose", Pose3d.struct)
          .publish();

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
      double error = m_simTargetHeight - m_simHeight;
      double maxDelta = Elevator.kSimLerpSpeed * 0.02; // dt assumed to be 20ms
      m_simHeight += Math.copySign(Math.min(Math.abs(error), maxDelta), error);
      
      m_simPoseArray.accept(
          new Pose3d[] {new Pose3d(0.0, 0.0, Units.inchesToMeters(m_simHeight), new Rotation3d())});
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

    setHeight(Elevator.kHeights.get(state));
  }

  /**
   * Check if the elevator is at the desired height
   *
   * @return boolean
   */
  public boolean atHeight() {
    return Math.abs(
            Elevator.kHeights.get(m_state)
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
    m_simTargetHeight = height;
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
  public void zero(PersistMode persistMode) {
    double previousOffset =
        Elevator.kLeftElevatorSparkMax.configAccessor.absoluteEncoder.getZeroOffset();

    Elevator.kLeftElevatorConfig.absoluteEncoder.zeroOffset(
        previousOffset + Elevator.kLeftElevatorSparkMax.getAbsoluteEncoder().getPosition());
    Elevator.kLeftElevatorSparkMax.configure(
        Elevator.kLeftElevatorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public Distance getSimEjectHeight() {
    return Inches.of(m_simHeight).plus(Inches.of(28));
  }
}
