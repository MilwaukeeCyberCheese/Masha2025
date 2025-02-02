package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  public ElevatorSubsystem() {
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
