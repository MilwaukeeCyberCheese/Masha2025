package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private final VisionSystemSim visionSim = new VisionSystemSim("main");
    private final AprilTagFieldLayout tagLayout;
    private final SwerveSubsystem drive;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private PhotonCameraSim cameraSim;
    private Transform3d robotToCamera;

    public VisionSubsystem(SwerveSubsystem drive) {
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        this.drive = drive;
        this.camera = new PhotonCamera("cameraName");

        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        photonEstimator = new PhotonPoseEstimator(
            tagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            robotToCamera);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            visionSim.addAprilTags(tagLayout);
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(0.25, 0.08);
            cameraProp.setFPS(20);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, cameraProp);
            visionSim.addCamera(cameraSim, robotToCamera);

            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(Robot.getInstance().m_robotContainer.m_drive.getPose()); // TODO: get pose from simdrive (maple-sim)
    }

    public Pose2d getPose() {
        Optional<EstimatedRobotPose> result = photonEstimator.update(camera.getLatestResult());
        return result.map(est -> est.estimatedPose.toPose2d()).orElse(drive.getPose());
    }

    @Override
    public void periodic() {
        var pose = this.getPose();
        if (pose != null) {
            drive.addVisionMeasurement(pose, Timer.getFPGATimestamp());
        }
    }
}
