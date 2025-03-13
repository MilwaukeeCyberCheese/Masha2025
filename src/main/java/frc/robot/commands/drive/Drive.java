package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

public class Drive extends Command {

  final SwerveSubsystem m_drive;
  final DoubleSupplier m_x;
  final DoubleSupplier m_y;
  final DoubleSupplier m_rotation;
  final BooleanSupplier m_slow;
  final Optional<DoubleSupplier> m_throttle;
  SwerveInputStream driveInput;

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
  public void initialize() {
    driveInput =
        SwerveInputStream.of(
                m_drive.getSwerveDrive(),
                () ->
                    m_x.getAsDouble()
                        * (m_slow.getAsBoolean()
                            ? DriveConstants.kDrivingSpeeds[1]
                            : DriveConstants.kDrivingSpeeds[0])
                        * m_throttle.orElse(() -> 1.0).getAsDouble(),
                () ->
                    m_y.getAsDouble()
                        * (m_slow.getAsBoolean()
                            ? DriveConstants.kDrivingSpeeds[1]
                            : DriveConstants.kDrivingSpeeds[0])
                        * m_throttle.orElse(() -> 1.0).getAsDouble())
            .withControllerRotationAxis(
                () ->
                    m_rotation.getAsDouble()
                        * (m_slow.getAsBoolean()
                            ? DriveConstants.kRotationSpeeds[1]
                            : DriveConstants.kRotationSpeeds[0])
                        * m_throttle.orElse(() -> 1.0).getAsDouble())
            .deadband(0.1)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drive.driveFieldOriented(driveInput.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
