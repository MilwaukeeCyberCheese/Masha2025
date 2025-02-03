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

  public ClimberSubsystem() {
    Climber.kClimberSparkMax.configure(
        Climber.kClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
