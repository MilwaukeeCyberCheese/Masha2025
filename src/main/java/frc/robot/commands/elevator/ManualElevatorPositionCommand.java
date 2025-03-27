package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ManualElevatorPositionCommand extends Command {

  private ElevatorSubsystem m_elevator;
  private DoubleSupplier m_change;

  public ManualElevatorPositionCommand(ElevatorSubsystem elevator, DoubleSupplier change) {
    m_elevator = elevator;
    m_change = change;

    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.setCustomTarget(m_elevator.getHeight() + m_change.getAsDouble());
  }
}
