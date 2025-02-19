package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChuteSubsystem extends SubsystemBase {

  public static enum ChutePositionState {
    UP,
    DOWN
  }

  private ChutePositionState m_positionState = ChutePositionState.UP;

  public ChuteSubsystem() {}

  public void drop() {
    m_positionState = ChutePositionState.DOWN;
  }

  @Override
  public void periodic() {
    // TODO: set the servo state
  }
}
