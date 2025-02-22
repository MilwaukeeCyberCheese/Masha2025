package frc.robot.subsystems.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class CoralHandlerSubsystemSim extends CoralHandlerSubsystem {

  public final IntakeSimulation m_intakeSim;
  private final AbstractDriveTrainSimulation m_drive;
  private final ElevatorSubsystemSim m_elevator;

  public CoralHandlerSubsystemSim(
      AbstractDriveTrainSimulation driveSim, ElevatorSubsystem elevator) {
    super();
    m_drive = driveSim;
    Rectangle intake = new Rectangle(.762, .245);
    intake.translate(new Vector2(0, .762));
    m_intakeSim = new IntakeSimulation("Coral", driveSim, intake, 1);
    m_elevator = (ElevatorSubsystemSim) elevator;
  }

  @Override
  public void periodic() {
    super.periodic();

    if (getState() == CoralHandlerState.GRAB) {
      m_intakeSim.startIntake();
    } else {
      m_intakeSim.stopIntake();
    }
    if (getState() == CoralHandlerState.RELEASE && m_intakeSim.obtainGamePieceFromIntake()) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new ReefscapeCoralOnFly(
                  m_drive.getSimulatedDriveTrainPose().getTranslation(),
                  new Translation2d(Inches.of(16.0), Inches.of(0)),
                  m_drive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  m_drive.getSimulatedDriveTrainPose().getRotation(),
                  m_elevator.getSimEjectHeight(),
                  MetersPerSecond.of(0.8), // eject speed
                  Radians.of(-Math.PI / 9)));
    }
    m_hasCoral = m_intakeSim.getGamePiecesAmount() != 0;
  }

  /** Intake a simulated coral from thin air, like magic */
  public void getSimCoral() {
    if (!m_intakeSim.addGamePieceToIntake()) {
      System.err.println("You already have a coral, you greedy bastard!");
    }
  }
}
