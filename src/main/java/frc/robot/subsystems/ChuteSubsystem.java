package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Chute;

public class ChuteSubsystem extends SubsystemBase {

  public enum ChuteState {
    UP,
    DOWN
  }

  private ChuteState m_state = ChuteState.UP;

  /** Creates a new ChuteSubsystem. */
  public ChuteSubsystem() {}

  @Override
  public void periodic() {
    setState(m_state);
  }

  public void setState(ChuteState state) {
    m_state = state;
    Chute.kServo.set(Chute.kPositions.get(m_state));
  }

  public ChuteState getState() {
    return m_state;
  }

  public void drop() {
    setState(ChuteState.DOWN);
  }

  public void raise() {
    setState(ChuteState.UP);
  }

  public boolean isDown() {
    return m_state == ChuteState.DOWN;
  }

  public boolean isUp() {
    return m_state == ChuteState.UP;
  }

  public void toggle() {
    if(isDown()) setState(ChuteState.UP);
    if(isUp()) setState(ChuteState.DOWN);;
  }
}
