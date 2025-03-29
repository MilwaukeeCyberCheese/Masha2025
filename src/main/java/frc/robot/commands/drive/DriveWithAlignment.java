package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Vision.HandlerCamera;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveInputStream;

public class DriveWithAlignment extends Command {

  private final SwerveSubsystem m_drive;
  private final DoubleSupplier m_x;
  private DoubleSupplier m_y;
  private final DoubleSupplier m_rotationX;
  private final DoubleSupplier m_rotationY;
  private final BooleanSupplier m_rotationMode;
  private final BooleanSupplier m_slowMode;
  private final Optional<DoubleSupplier> m_throttle;
  private SwerveInputStream rotationMode;
  private SwerveInputStream headingMode;

  private final PhotonCamera m_camera = new PhotonCamera(HandlerCamera.kCameraName);

  private final PIDController m_yController = new PIDController(0.005, 0, 0);

  public DriveWithAlignment(
      SwerveSubsystem drive,
      DoubleSupplier x,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY,
      BooleanSupplier rotationMode,
      BooleanSupplier slow,
      Optional<DoubleSupplier> throttle) {
    m_drive = drive;
    m_x = x;
    m_rotationX = rotationX;
    m_rotationY = rotationY;
    m_rotationMode = rotationMode;
    m_slowMode = slow;
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
                m_y)
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

    m_yController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target = m_camera.getLatestResult().getBestTarget();

    m_y = () -> (target != null) ? m_yController.calculate(target.getYaw()) : 0.0;

    m_drive.driveFieldOriented(m_rotationMode.getAsBoolean() ? rotationMode.get() : headingMode.get());
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
