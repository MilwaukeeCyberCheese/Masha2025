package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

// TODO: add sim support
public class ClimberSubsystem extends SubsystemBase {

  public static enum ClimberState {
    WAITING,
    STOWED,
    CLIMB
  }

  private ClimberState m_state = ClimberState.STOWED;

  public ClimberSubsystem() {
    Climber.kClimberSparkMax.configure(
        Climber.kClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setState(m_state);
  }

  // TODO: add logic for limits
  /**
   * Set the state of the climber
   *
   * @param state options from {@link ClimberState}
   */
  public void setState(ClimberState state) {
    m_state = state;
    setPosition(Climber.kClimberPositions.get(state));
  }

  /**
   * Set the position of the climber
   *
   * @param position
   */
  private void setPosition(double position) {
    Climber.kClimberController.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Check if the climber is at the desired position
   *
   * @return boolean
   */
  public boolean atState() {
    return Math.abs(
            Climber.kClimberPositions.get(m_state)
                - Climber.kClimberSparkMax.getAbsoluteEncoder().getPosition())
        < Climber.kClimberTolerance;
  }

  /**
   * Get the current state of the climber
   *
   * @return {@link ClimberState}
   */
  public ClimberState getState() {
    return m_state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    log();
  }

  // TODO; add logging code
  public void log() {}
}
