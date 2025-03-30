/*
 * All measurements should be assumed to be in meters and radians unless otherwise noted.
 */

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignWithReef extends Command {
  private final SwerveSubsystem m_drive;
  private final Transform3d m_offset;
  private final String m_camName;

  // TODO: tune, should be similar to pathplanner values
  // TODO: after tuning, move to constants
  private PIDController m_xController = new PIDController(2.5, 0, 0);
  private PIDController m_yController = new PIDController(2.5, 0, 0);
  private PIDController m_thetaController = new PIDController(0.0003, 0, 0);

  /**
   * Snap to a certain angle relative to the field, and then line up with the apriltag indicated in
   * the x and y plane
   *
   * @param driveSubsystem
   * @param cameraSubsystem
   * @param id id of the apriltag to orient to
   */
  public AlignWithReef(SwerveSubsystem driveSubsystem, Transform3d offset, String camName) {
    m_drive = driveSubsystem;
    m_offset = offset;
    m_camName = camName;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_xController.reset();
    m_xController.setSetpoint(m_offset.getX());
    // TODO: find error tolerance
    m_xController.setTolerance(0.1);

    m_yController.reset();
    m_yController.setSetpoint(m_offset.getY());
    // TODO: find error tolerance
    m_yController.setTolerance(0.1);

    m_thetaController.reset();
    m_thetaController.setSetpoint(m_offset.getRotation().getZ());
    // TODO: find error tolerance
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private PhotonTrackedTarget getClosestAprilTag(PhotonCamera cam) {
    PhotonTrackedTarget bestTarget = null;
    double bestDistance = Double.MAX_VALUE;

    var result = cam.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    List<PhotonTrackedTarget> reefTargets = new LinkedList<>();
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId()
              >= (DriverStation.getAlliance().equals(DriverStation.Alliance.Red) ? 6 : 11)
          && target.getFiducialId()
              <= (DriverStation.getAlliance().equals(DriverStation.Alliance.Red) ? 22 : 17)) {
        reefTargets.add(target);
      }
    }

    if (reefTargets.size() > 0) {
      for (PhotonTrackedTarget target : reefTargets) {
        double targetDistance =
            Math.sqrt(
                Math.pow(target.bestCameraToTarget.getX(), 2)
                    + Math.pow(target.bestCameraToTarget.getY(), 2));
        if (targetDistance < bestDistance) {
          bestDistance = targetDistance;
          bestTarget = target;
        }
      }
    }
    return bestTarget;
  }

  @Override
  public void execute() {
    double xOutput = 0;
    double yOutput = 0;
    double thetaOutput = 0;

    PhotonTrackedTarget target = getClosestAprilTag(new PhotonCamera(m_camName));
    // Check if target is present
    if (target != null) {
      // If target is present, get transform and calculate outputs
      Transform3d transform = target.getBestCameraToTarget();
      System.out.println(transform.toString());
      xOutput = m_xController.calculate(transform.getX());
      yOutput = m_yController.calculate(transform.getY());
      thetaOutput = m_thetaController.calculate(transform.getRotation().getZ());
    }

    m_drive.drive(new ChassisSpeeds(xOutput, yOutput, thetaOutput));
  }

  @Override
  public boolean isFinished() {
    return m_xController.atSetpoint()
        && m_yController.atSetpoint()
        && m_thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(new ChassisSpeeds(0, 0, 0));
  }
}
