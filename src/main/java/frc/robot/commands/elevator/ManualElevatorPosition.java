package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ManualElevatorPosition extends Command {

  private ElevatorSubsystem m_elevator;
  private DoubleSupplier m_change;

  public ManualElevatorPosition(ElevatorSubsystem elevator, DoubleSupplier change) {
    m_elevator = elevator;
    m_change = change;

    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.customAdjust(m_change.getAsDouble() * -0.25);
  }
}
