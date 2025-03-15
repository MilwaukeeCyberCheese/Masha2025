package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Chute;

public class ChuteSubsystem extends SubsystemBase {

  public ChuteSubsystem() {
    Chute.kServo.set(Chute.kUp);
  }

  public void drop() {
    Chute.kServo.set(Chute.kDown);
  }
}
