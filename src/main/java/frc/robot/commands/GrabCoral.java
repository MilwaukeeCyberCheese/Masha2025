package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Handler.Coral;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.utils.Stopwatch;

public class GrabCoral extends Command {

  private final CoralHandlerSubsystem m_coralHandlerSubsystem;
  private final Stopwatch m_timer = new Stopwatch();
  private boolean m_finished = false;

  /**
   * Runs coral handler until coral is detected, then continues running for offset
   *
   * @param coralHandlerSubsystem
   */
  public GrabCoral(CoralHandlerSubsystem coralHandlerSubsystem) {
    m_coralHandlerSubsystem = coralHandlerSubsystem;

    addRequirements(coralHandlerSubsystem);
  }

  @Override
  public void initialize() {
    if (m_coralHandlerSubsystem.hasCoral()) {
      m_finished = true;
      return;
    }

    m_coralHandlerSubsystem.grab();
    m_timer.reset();
  }

  @Override
  public void execute() {
    if (m_coralHandlerSubsystem.hasCoral() && !m_timer.isRunning()) {
      m_timer.start();
    }

    m_finished = m_timer.isRunning() ? m_timer.getTime() > Coral.kDetectionDelayTimeMS : false;
  }

  @Override
  public void end(boolean interrupted) {
    m_coralHandlerSubsystem.inactive();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
