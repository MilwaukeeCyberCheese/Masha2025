package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Handler.Coral;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.utils.Stopwatch;

public class GrabCoralCommand extends Command {

  private final CoralHandlerSubsystem m_coralHandlerSubsystem;
  private final Stopwatch timer = new Stopwatch();

  public GrabCoralCommand(CoralHandlerSubsystem coralHandlerSubsystem) {
    m_coralHandlerSubsystem = coralHandlerSubsystem;

    addRequirements(coralHandlerSubsystem);
  }

  @Override
  public void initialize() {
    m_coralHandlerSubsystem.grab();
    timer.reset();
  }

  @Override
  public void execute() {
    if (m_coralHandlerSubsystem.hasCoral()) {
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_coralHandlerSubsystem.inactive();
  }

  @Override
  public boolean isFinished() {
    return timer.isRunning() ? timer.getTime() > Coral.kDetectionDelayTimeMS : false;
  }
}
