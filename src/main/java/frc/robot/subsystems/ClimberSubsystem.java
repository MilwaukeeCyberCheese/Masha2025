package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {

  public ClimberSubsystem() {
    // Initialize motors, sensors, etc. here
  }

  // Methods to set motor speeds, etc. go here

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    log();
  }

  public void log() {
    // Log sensor data, etc. here
  }

  


}
