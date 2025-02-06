package frc.robot.subsystems.sim;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSubsystemSim extends ElevatorSubsystem {

  private double m_simHeight = 0.0;
  private double m_simTargetHeight = 0.0;

  StructArrayPublisher<Pose3d> m_simPoseArray =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Simulation/ElevatorPose", Pose3d.struct)
          .publish();

  public ElevatorSubsystemSim() {
    super();
  }

  // Methods to set motor speeds, etc. go here

  @Override
  public void periodic() {
    super.periodic();
    double error = m_simTargetHeight - m_simHeight;
    double maxDelta = Elevator.kSimLerpSpeed * 0.02; // dt assumed to be 20ms
    m_simHeight += Math.copySign(Math.min(Math.abs(error), maxDelta), error);

    m_simPoseArray.accept(
        new Pose3d[] {new Pose3d(0.0, 0.0, Units.inchesToMeters(m_simHeight), new Rotation3d())});
  }

  @Override
  public void setState(ElevatorState state) {
    super.setState(state);
    m_simTargetHeight = m_height;
    SmartDashboard.putNumber("Elevator/SimTargetHeight", m_height);
  }

  public Distance getSimEjectHeight() {
    return Inches.of(m_simHeight).plus(Inches.of(28));
  }
}
