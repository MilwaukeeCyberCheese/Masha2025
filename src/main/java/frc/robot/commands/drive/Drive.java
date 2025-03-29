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
  final DoubleSupplier m_rotationX;
  final DoubleSupplier m_rotationY;
  final BooleanSupplier m_rotationMode;
  final BooleanSupplier m_slowMode;
  final Optional<DoubleSupplier> m_throttle;
  SwerveInputStream rotationMode;
  SwerveInputStream headingMode;

  /**
   * Drives the robot using field-oriented control
   *
   * <p>Recall that the coordinate system used is NWU (North, West, Up)
   *
   * <p>See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
   *
   * @param drive
   * @param x
   * @param y
   * @param rotationX
   * @param rotationY
   * @param rotationMode false is heading, true is rotation
   * @param slowMode
   * @param throttle
   */
  public Drive(
      SwerveSubsystem drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY,
      BooleanSupplier rotationMode,
      BooleanSupplier slowMode,
      Optional<DoubleSupplier> throttle) {
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_rotationX = rotationX;
    m_rotationY = rotationY;
    m_rotationMode = rotationMode;
    m_slowMode = slowMode;
    m_throttle = throttle;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationMode =
        SwerveInputStream.of(
                m_drive.getSwerveDrive(),
                () ->
                    m_x.getAsDouble()
                        * (m_slowMode.getAsBoolean()
                            ? DriveConstants.kDrivingSpeeds[1]
                            : DriveConstants.kDrivingSpeeds[0])
                        * m_throttle.orElse(() -> 1.0).getAsDouble(),
                () ->
                    m_y.getAsDouble()
                        * (m_slowMode.getAsBoolean()
                            ? DriveConstants.kDrivingSpeeds[1]
                            : DriveConstants.kDrivingSpeeds[0])
                        * m_throttle.orElse(() -> 1.0).getAsDouble())
            .deadband(0.1)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true)
            .withControllerRotationAxis(
                () ->
                    m_rotationX.getAsDouble()
                        * (m_slowMode.getAsBoolean()
                            ? DriveConstants.kRotationSpeeds[1]
                            : DriveConstants.kRotationSpeeds[0])
                        * m_throttle.orElse(() -> 1.0).getAsDouble());

    headingMode =
        rotationMode.copy().headingWhile(true).withControllerHeadingAxis(m_rotationX, m_rotationY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveFieldOriented(
        m_rotationMode.getAsBoolean() ? rotationMode.get() : headingMode.get());
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
