package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;
import java.util.Optional;

// TODO: add sim support
public class ClimberSubsystem extends SubsystemBase {

  public static enum ClimberState {
    WAITING,
    STOWED,
    CLIMB,
    CUSTOM
  }

  private ClimberState state = ClimberState.STOWED;
  private double position = 0;
  private Optional<Double> customPosition = Optional.empty();

  public ClimberSubsystem() {
    Climber.CLIMBER_SPARK_MAX.configure(
        Climber.CLIMBER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setState(state);
  }

  @Override
  public void periodic() {
    Climber.CLIMBER_CONTROLLER.setReference(position, ControlType.kPosition);

    log();
  }

  public void log() {
    SmartDashboard.putNumber(
        "Climber Position", Climber.CLIMBER_SPARK_MAX.getAbsoluteEncoder().getPosition());
  }

  // TODO: add logic for limits
  /**
   * Set the state of the climber
   *
   * @param state options from {@link ClimberState}
   */
  public void setState(ClimberState state) {
    if (state == ClimberState.CUSTOM && customPosition.isEmpty()) {
      return;
    }

    this.state = state;

    position =
        this.state == ClimberState.CUSTOM ? customPosition.get() : Climber.POSITIONS.get(this.state);
  }

  /**
   * Get the current state of the climber
   *
   * @return {@link ClimberState}
   */
  public ClimberState getState() {
    return state;
  }

  /**
   * Set the custom position of the climber Does not change the state of the climber
   *
   * @param position
   */
  public void setCustomPosition(double position) {
    customPosition = Optional.of(position);
  }

  /**
   * Get the current position of the climber
   *
   * @return double
   */
  public double getPosition() {
    return Climber.CLIMBER_SPARK_MAX.getAbsoluteEncoder().getPosition();
  }

  /**
   * Check if the climber is at the desired position
   *
   * @return boolean
   */
  public boolean atPosition() {
    return Math.abs(
            Climber.POSITIONS.get(state)
                - Climber.CLIMBER_SPARK_MAX.getAbsoluteEncoder().getPosition())
        < Climber.CLIMBER_TOLERANCE;
  }

  /** Set climber state to waiting */
  public void waiting() {
    setState(ClimberState.WAITING);
  }

  /** Set climber state to stowed */
  public void stowed() {
    setState(ClimberState.STOWED);
  }

  /** Set climber state to climb */
  public void climb() {
    setState(ClimberState.CLIMB);
  }
}
