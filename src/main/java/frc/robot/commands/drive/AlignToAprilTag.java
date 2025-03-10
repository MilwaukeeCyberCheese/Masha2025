package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import org.photonvision.PhotonUtils;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class AlignToAprilTag extends Command {

    private final SwerveSubsystem drive;
    private final IntSupplier aprilTag;
    private final Supplier<Translation2d> translation;

    private final PIDController xController = new PIDController(3.5, 0, 0);
    private final PIDController yController = new PIDController(3.5, 0, 0);
    private final PIDController thetaController = new PIDController(Math.PI/4, 0, 0);

    {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public AlignToAprilTag(SwerveSubsystem drive, IntSupplier aprilTag, Supplier<Translation2d> translation) {
        this.drive = drive;
        this.aprilTag = aprilTag;
        this.translation = translation;
    }

    @Override
    public void execute() {
        var tagPose = Vision.fieldLayout.getTagPose(this.aprilTag.getAsInt());

        var xOutput = 0.0;
        var yOutput = 0.0;

        if (tagPose.isPresent()) {
            final var robotPose = this.drive.getPose().getTranslation();
            final var desiredPose = tagPose.get()
                    .getTranslation()
                    .toTranslation2d()
                    .plus(this.translation.get());
            System.out.println(desiredPose);

            xOutput = xController.calculate(robotPose.getX(), desiredPose.getX());
            yOutput = yController.calculate(robotPose.getY(), desiredPose.getX());
        }

        // snap to theta using gyro
        var thetaOutput = thetaController.calculate(Math.toRadians(Constants.Sensors.gyro.getAngle()));
        System.out.printf("x: %f, y: %f, t: %f\n", xOutput, yOutput, thetaOutput);
        this.drive.driveFieldOriented(new ChassisSpeeds(xOutput, yOutput, thetaOutput));
    }
}
