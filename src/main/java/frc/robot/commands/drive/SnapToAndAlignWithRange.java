package frc.robot.commands.drive;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SnapToAndAlignWithRange extends Command {
    private final SwerveSubsystem m_drive;
    private final IntSupplier m_id;

    private PIDController m_xController = new PIDController(0.005,
            0,0);
    private PIDController m_yController = new PIDController(0.005,0,0);
    private PIDController m_thetaController = new PIDController(0.0003, 0,0);

    /**
     * Snap to a certain angle relative to the field, and then line up with
     * the apriltag indicated in the x and y plane
     * 
     * @param driveSubsystem
     * @param cameraSubsystem
     * @param id              id of the apriltag to orient to
     */
    public SnapToAndAlignWithRange(SwerveSubsystem driveSubsystem,
            IntSupplier id) {
        m_drive = driveSubsystem;
        m_id = id;
        addRequirements( m_drive);
    }

    @Override
    public void initialize() {
        m_xController.reset();
        m_xController.setSetpoint(0.2);

        m_yController.reset();
        m_yController.setSetpoint(0.2);

        m_thetaController.reset();
        m_thetaController.setSetpoint(180);
        m_thetaController.enableContinuousInput(-180, 180);
    }

     public PhotonTrackedTarget getAprilTag(int id, PhotonCamera cam) {
        var result = cam.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) {
                return target;
            }
        }
        return null;
    }

    @Override
    public void execute() {
        double xOutput = 0;
        double yOutput = 0;
        double thetaOutput = 0;
        // TODO: figure out how range works and make it accurate

        PhotonCamera camera = new PhotonCamera(Vision.kCameraName);

        PhotonTrackedTarget target = getAprilTag(17, camera);
        // check if target is present
        if (target != null) {
            Transform3d val = target.getBestCameraToTarget();
            System.out.println(val);
            xOutput = m_xController.calculate(val.getX());
            yOutput = m_yController.calculate(val.getY());
            thetaOutput = m_thetaController.calculate(val.getZ());
            
        }

        // snap to theta using gyro
        // thetaOutput = m_thetaController.calculate(Math.toRadians(Constants.Sensors.gyro.getAngle()));
        m_drive.drive(new ChassisSpeeds(xOutput, yOutput, thetaOutput));

    }

    @Override
    public boolean isFinished() {
        // return m_xController.atSetpoint();
        return false;
    }
}