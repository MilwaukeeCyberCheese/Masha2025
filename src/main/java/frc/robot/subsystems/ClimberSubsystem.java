package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

// TODO: add sim support
public class ClimberSubsystem extends SubsystemBase {

  public static enum ClimberState {
    WAITING,
    STOWED,
    CLIMB
  }

  public ClimberSubsystem() {
    // Initialize motors, sensors, etc. here
  }

  // TODO: add logic for limits
  public void setState(ClimberState state) {
    setPosition(Climber.kClimberPositions.get(state));
  }

  private void setPosition(double position) {
    Climber.kClimberController.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    log();
  }

  // TODO; add logging code
  public void log() {}
}
