package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;

  public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(this.elevatorSubsystem);
  }

  @Override
  public void execute() {
    elevatorSubsystem.setCustomTarget(elevatorSubsystem.getHeight() - Elevator.ZEROING_STEP);
    elevatorSubsystem.setState(ElevatorSubsystem.ElevatorState.CUSTOM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atBottom();
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.zero();
  }
}
