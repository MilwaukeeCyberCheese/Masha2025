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

  private AlgaeHandlerPositionState positionState = AlgaeHandlerPositionState.STOWED;
  private AlgaeHandlerIntakeState intakeState = AlgaeHandlerIntakeState.STOPPED;

  private double position;
  private double speed;

  private Optional<Double> customPosition = Optional.empty();
  private Optional<Double> customSpeed = Optional.empty();

  public AlgaeHandlerSubsystem() {
    Algae.kPositionSparkMax.configure(
        Algae.kPositionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Algae.kIntakeSparkMax.configure(
        Algae.kIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setPositionState(AlgaeHandlerPositionState.STOWED);
    setSpeedState(AlgaeHandlerIntakeState.STOPPED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Algae.kPositionController.setReference(position, ControlType.kPosition);
    Algae.kIntakeController.setReference(speed, ControlType.kVelocity);

    log();
  }

  // TODO; add logging code
  public void log() {
    SmartDashboard.putNumber(
        "Algae Handler Position", Algae.kPositionSparkMax.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber(
        "Algae Handler Intake Speed", Algae.kIntakeSparkMax.getEncoder().getVelocity());
  }

  // TODO: add logic for limits
  /**
   * Set the state of the algae handler's position
   *
   * @param state options from {@link AlgaeHandlerPositionState}
   */
  public void setPositionState(AlgaeHandlerPositionState state) {
    if (state == AlgaeHandlerPositionState.CUSTOM && customPosition.isEmpty()) {
      return;
    }

    positionState = state;

    position =
        positionState == AlgaeHandlerPositionState.CUSTOM
            ? customPosition.get()
            : Algae.kPositions.get(state);
  }

  /**
   * Get the current state of the algae handler's position
   *
   * @return {@link AlgaeHandlerPositionState}
   */
  public AlgaeHandlerPositionState getPositionState() {
    return positionState;
  }

  /**
   * Set custom position for the algae handler This does not change the state of the algae handler
   *
   * @param position double
   */
  public void setCustomPosition(double position) {
    customPosition = Optional.of(position);
  }

  /**
   * Get the position of the algae handler
   *
   * @return double
   */
  public double getPosition() {
    return position;
  }

  /**
   * Check if the algae handler is at the desired position
   *
   * @return boolean
   */
  public boolean atPosition() {
    return Math.abs(
            Algae.kPositions.get(positionState)
                - Algae.kPositionSparkMax.getAbsoluteEncoder().getPosition())
        < Algae.kPositionTolerance;
  }

  /**
   * Set the state of the algae handler's intake
   *
   * @param state options from {@link AlgaeHandlerIntakeState}
   */
  public void setSpeedState(AlgaeHandlerIntakeState state) {
    if (state == AlgaeHandlerIntakeState.CUSTOM && customSpeed.isEmpty()) {
      return;
    }

    intakeState = state;

    speed =
        intakeState == AlgaeHandlerIntakeState.CUSTOM
            ? customSpeed.get()
            : Algae.kSpeeds.get(state);
  }

  /**
   * Get the current state of the algae handler's intake
   *
   * @return {@link AlgaeHandlerIntakeState}
   */
  public AlgaeHandlerIntakeState getSpeedState() {
    return intakeState;
  }

  /**
   * Set custom speed for the algae handler's intake
   *
   * @param speed double
   */
  public void setCustomSpeed(double speed) {
    customSpeed = Optional.of(speed);
  }

  /**
   * Get the speed of the algae handler's intake
   *
   * @return double
   */
  public double getSpeed() {
    return speed;
  }

  /**
   * Get whether the algae handler's intake is at the desired state
   *
   * @return boolean
   */
  public boolean atSpeed() {
    return Math.abs(
            Algae.kSpeeds.get(intakeState) - Algae.kIntakeSparkMax.getEncoder().getVelocity())
        < Algae.kIntakeTolerance;
  }
}
