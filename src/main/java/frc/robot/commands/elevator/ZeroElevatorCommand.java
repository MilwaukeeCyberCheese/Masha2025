package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorCommand extends Command {

  private final ElevatorSubsystem m_elevatorSubsystem;

  public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;

    addRequirements(m_elevatorSubsystem);
  }

  @Override
  public void execute() {
    m_elevatorSubsystem.setCustomTarget(m_elevatorSubsystem.getHeight() - Elevator.kZeroingStep);
    m_elevatorSubsystem.setState(ElevatorSubsystem.ElevatorState.CUSTOM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.atBottom();
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.zero();
  }
}
