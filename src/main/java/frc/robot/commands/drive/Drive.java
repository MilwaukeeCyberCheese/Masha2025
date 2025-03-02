package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends Command {

  SwerveSubsystem m_drive;
  DoubleSupplier m_x;
  DoubleSupplier m_y;
  DoubleSupplier m_rotation;
  BooleanSupplier m_slow;
  Optional<DoubleSupplier> m_throttle;

  public Drive(
      SwerveSubsystem drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rotation,
      BooleanSupplier slow,
      Optional<DoubleSupplier> throttle) {
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_rotation = rotation;
    m_slow = slow;
    m_throttle = throttle;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
