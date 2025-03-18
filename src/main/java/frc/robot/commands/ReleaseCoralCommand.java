package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Handler.Coral;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.utils.Stopwatch;

public class ReleaseCoralCommand extends Command {

  private final CoralHandlerSubsystem m_coralHandlerSubsystem;
  private final Stopwatch timer = new Stopwatch();

  public ReleaseCoralCommand(CoralHandlerSubsystem coralHandlerSubsystem) {
    m_coralHandlerSubsystem = coralHandlerSubsystem;

    addRequirements(coralHandlerSubsystem);
  }

  @Override
  public void initialize() {
    m_coralHandlerSubsystem.release();
    timer.reset();
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    m_coralHandlerSubsystem.inactive();
  }

  @Override
  public boolean isFinished() {
    return timer.getTime() > Coral.kReleaseTimeMS;
  }
}
