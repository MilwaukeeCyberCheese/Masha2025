package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MoveToPose extends Command {

    private final SwerveSubsystem drive;
    private final Supplier<Pose2d> pose;

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    private final ProfiledPIDController driveController = new ProfiledPIDController(0.8, 0., 0., new TrapezoidProfile.Constraints(3.8, 3.));
    private final ProfiledPIDController thetaController = new ProfiledPIDController(4., 0., 0., new TrapezoidProfile.Constraints(2. * Math.PI, 8.));
    private final double ffMinRadius = 0.05;
    private final double ffMaxRadius = 0.1;

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;

    private final FieldObject2d debugPos;

    {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public MoveToPose(SwerveSubsystem drive, Supplier<Pose2d> pose) {
        this.drive = drive;
        this.pose = pose;
        this.debugPos = this.drive.getSwerveDrive().field.getObject("Auto Alignment/Desired");
    }

    @Override
    public void initialize() {
        System.out.println("move to pose cmd now");

        final var robotPose = this.drive.getPose();
        final var targetPose = this.pose.get();
        final var fieldVelocity = this.drive.getFieldVelocity();

        final var linearFieldVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        driveController.reset(
                robotPose.getTranslation().getDistance(targetPose.getTranslation()),
                Math.min(
                        0.0,
                        -linearFieldVelocity
                                .rotateBy(
                                        targetPose
                                                .getTranslation()
                                                .minus(robotPose.getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                robotPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = robotPose.getTranslation();
    }

    @Override
    public void execute() {
        final var robotPose = this.drive.getPose();
        final var targetPose = this.pose.get();
        this.debugPos.setPose(targetPose);

        // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/commands/DriveToPose.java
        // Calculate drive speed
        double currentDistance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0., 1.
        );

        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity
        );

        double driveVelocityScalar =
                driveController.getSetpoint().velocity * ffScaler
                        + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) {
            driveVelocityScalar = 0.0;
        }

        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                        Math.atan2(
                                robotPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                                robotPose.getTranslation().getX() - targetPose.getTranslation().getX())))
                .transformBy(new Transform2d(driveController.getSetpoint().position, 0.0, Rotation2d.kZero))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(robotPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) {
            thetaVelocity = 0.0;
        }

        Translation2d driveVelocity =
                new Pose2d(
                        Translation2d.kZero,
                        new Rotation2d(
                                Math.atan2(
                                        robotPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                                        robotPose.getTranslation().getX() - targetPose.getTranslation().getX())))
                        .transformBy(new Transform2d(driveVelocityScalar, 0.0, Rotation2d.kZero))
                        .getTranslation();

        // Scale feedback velocities by input ff
        final double linearS = linearFF.get().getNorm() * 3.0;
        final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
        driveVelocity = driveVelocity.interpolate(
                linearFF.get().times(Constants.DriveConstants.kMaxSpeedMetersPerSecond),
                linearS
        );
        thetaVelocity = MathUtil.interpolate(
                thetaVelocity,
                omegaFF.getAsDouble() * Constants.DriveConstants.kMaxAngularSpeed,
                thetaS
        );

        // Command speeds
        drive.driveFieldOriented(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaVelocity,
                robotPose.getRotation()
        ));

    }

    @Override
    public boolean isFinished() {
        System.out.println("mtp is finished?");
        return this.driveController.atGoal() && this.thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        this.debugPos.close();
    }
}
