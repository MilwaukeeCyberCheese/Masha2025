package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import swervelib.SwerveInputStream;

public class DriveCOR extends Command {

  final SwerveSubsystem m_drive;
  final DoubleSupplier m_x;
  final DoubleSupplier m_y;
  final DoubleSupplier m_rotation;
  final BooleanSupplier m_slow;
  final Optional<DoubleSupplier> m_throttle;
  final Optional<IntSupplier> m_cor;
  SwerveInputStream driveInput;

  /**
   * Creates a new Drive COR command.
   *
   * @param drive The drive subsystem this command will run on
   * @param x
   * @param y
   * @param rotation
   * @param slow
   * @param throttle Throttle multiplier (Optional)
   * @param cor Center of rotation, should be degrees with 0 being up, increasing clockwise
   *     (Optional)
   */
  public DriveCOR(
      SwerveSubsystem drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rotation,
      BooleanSupplier slow,
      Optional<DoubleSupplier> throttle,
      Optional<IntSupplier> cor) {
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_rotation = rotation;
    m_slow = slow;
    m_throttle = throttle;
    m_cor = cor;
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
    // Convert COR to radians, keeping in mind that 0 is up and increasing clockwise
    // This could be done in a much better way, but it works ¯\_(ツ)_/¯
    Rotation2d cor =
        switch (m_cor.orElse(() -> -1).getAsInt()) {
          case 90 -> Rotation2d.fromDegrees(0);
          case 45 -> Rotation2d.fromDegrees(45);
          case 0 -> Rotation2d.fromDegrees(90);
          case 315 -> Rotation2d.fromDegrees(135);
          case 270 -> Rotation2d.fromDegrees(180);
          case 225 -> Rotation2d.fromDegrees(225);
          case 180 -> Rotation2d.fromDegrees(270);
          case 135 -> Rotation2d.fromDegrees(315);
          default -> null;
        };

    m_drive.driveFieldOriented(driveInput.get(), cor);
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
