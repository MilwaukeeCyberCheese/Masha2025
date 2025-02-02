package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Sensors;
import frc.robot.Constants.HandlerConstants;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CoralHandlerSubsystem extends SubsystemBase {
  // two neo 550s, 9:1 gearing on both, going in opposite directions.
  // looking on from the top, the right one going clockwise is intake, and
  // counterclockwise is
  // outtake.
  // vice versa for the other

  public final IntakeSimulation m_intakeSim;

  public enum Mode {
    kInactive,
    kIntake,
    kOuttake,
  }

  private Mode m_mode = Mode.kInactive;
  private boolean m_hasCoral = false;

  public CoralHandlerSubsystem(AbstractDriveTrainSimulation driveSim) {
    m_intakeSim = new IntakeSimulation("Coral", driveSim, new Rectangle(.762, 1.007), 1);
  }

  public Mode getMode() {
    return m_mode;
  }

  public boolean hasCoral() {
    return m_hasCoral;
  }

  public void setMode(Mode mode) {
    m_mode = mode;
    switch (mode) {
      case kInactive:
        setSpeed(0);
        break;
      case kIntake:
        setSpeed(HandlerConstants.kIntakeSpeed);
        break;
      case kOuttake:
        setSpeed(HandlerConstants.kOuttakeSpeed);
        break;
    }
  }

  private void setSpeed(double speed) {
    HandlerConstants.m_left.set(speed);
    HandlerConstants.m_right.set(speed);
  }

  @Override
  public void periodic() {
    Robot.getInstance();
    if (Robot.isReal()) {
      m_hasCoral = Sensors.handlerDistanceSensor.getRange(Unit.kInches) < 5;
    } else {
      if (m_mode == Mode.kIntake) {
        m_intakeSim.startIntake();
      } else {
        m_intakeSim.stopIntake();
      }
      m_hasCoral = m_intakeSim.getGamePiecesAmount() != 0;
    }
  }

  public void intake() {
    setMode(Mode.kIntake);
  }

  public void outtake() {
    setMode(Mode.kOuttake);
  }
}
