package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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
    // This method will be called once per scheduler run
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
    m_simHeight = height;
  }

  // TODO: test this and make sure the
  // disabling works
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

  public Translation2d getSimEjectPosition() {
    return new Translation2d(Inches.of(m_simHeight), Inches.of(0.0));
  }
}
