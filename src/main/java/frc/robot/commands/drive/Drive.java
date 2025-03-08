package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

public class Drive extends Command {

  final SwerveSubsystem drive;
  final DoubleSupplier x;
  final DoubleSupplier y;
  final DoubleSupplier rotation;
  final BooleanSupplier slow;
  final Optional<DoubleSupplier> throttle;
  SwerveInputStream driveInput;

  public Drive(
      SwerveSubsystem drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rotation,
      BooleanSupplier slow,
      Optional<DoubleSupplier> throttle) {
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.rotation = rotation;
    this.slow = slow;
    this.throttle = throttle;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveInput =
        SwerveInputStream.of(
                drive.getSwerveDrive(),
                () ->
                    x.getAsDouble()
                        * (slow.getAsBoolean()
                            ? DriveConstants.DRIVING_SPEEDS[1]
                            : DriveConstants.DRIVING_SPEEDS[0])
                        * throttle.orElse(() -> 1.0).getAsDouble(),
                () ->
                    y.getAsDouble()
                        * (slow.getAsBoolean()
                            ? DriveConstants.DRIVING_SPEEDS[1]
                            : DriveConstants.DRIVING_SPEEDS[0])
                        * throttle.orElse(() -> 1.0).getAsDouble())
            .withControllerRotationAxis(
                () ->
                    rotation.getAsDouble()
                        * (slow.getAsBoolean()
                            ? DriveConstants.ROTATION_SPEEDS[1]
                            : DriveConstants.ROTATION_SPEEDS[0])
                        * throttle.orElse(() -> 1.0).getAsDouble())
            .deadband(0.1)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveFieldOriented(driveInput);
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
