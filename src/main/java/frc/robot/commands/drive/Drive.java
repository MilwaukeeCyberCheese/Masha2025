package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

public class Drive extends Command {

  final SwerveSubsystem m_drive;
  final DoubleSupplier m_x;
  final DoubleSupplier m_y;
  final DoubleSupplier m_rotation;
  final BooleanSupplier m_fieldCentric;
  final BooleanSupplier m_slowMode;
  SwerveInputStream inputStream;

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
   * @param rotation
   * @param fieldCentric
   * @param slowMode
   */
  public Drive(
      SwerveSubsystem drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rotation,
      BooleanSupplier fieldCentric,
      BooleanSupplier slowMode) {
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_rotation = rotation;
    m_fieldCentric = fieldCentric;
    m_slowMode = slowMode;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inputStream =
        SwerveInputStream.of(
                m_drive.getSwerveDrive(),
                () ->
                    m_x.getAsDouble()
                        * (m_slowMode.getAsBoolean()
                            ? DriveConstants.kDrivingSpeeds[1]
                            : DriveConstants.kDrivingSpeeds[0]),
                () ->
                    m_y.getAsDouble()
                        * (m_slowMode.getAsBoolean()
                            ? DriveConstants.kDrivingSpeeds[1]
                            : DriveConstants.kDrivingSpeeds[0]))
            .deadband(0.1)
            .cubeTranslationControllerAxis(true)
            .cubeRotationControllerAxis(true)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true)
            .withControllerRotationAxis(
                () ->
                    m_rotation.getAsDouble()
                        * (m_slowMode.getAsBoolean()
                            ? DriveConstants.kRotationSpeeds[1]
                            : DriveConstants.kRotationSpeeds[0]));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_fieldCentric.getAsBoolean()) {
      m_drive.driveFieldOriented(inputStream.get());

    } else {
      m_drive.drive(new ChassisSpeeds(-inputStream.get().vxMetersPerSecond, -inputStream.get().vyMetersPerSecond, inputStream.get().omegaRadiansPerSecond));
    }
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
