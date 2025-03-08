package frc.robot.subsystems.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Sensors;
import frc.robot.Robot;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class CoralHandlerSubsystemSim extends CoralHandlerSubsystem {

  public final IntakeSimulation intakeSim;
  private final AbstractDriveTrainSimulation drive;
  private final ElevatorSubsystemSim elevator;

  public CoralHandlerSubsystemSim(
      AbstractDriveTrainSimulation driveSim, ElevatorSubsystem elevator) {
    super();
    drive = driveSim;
    Rectangle intake = new Rectangle(.762, .245);
    intake.translate(new Vector2(0, .762));
    intakeSim = new IntakeSimulation("Coral", driveSim, intake, 1);
    this.elevator = (ElevatorSubsystemSim) elevator;
  }

  @Override
  public void periodic() {
    super.periodic();
    if (Robot.isReal()) {
      hasCoral = Sensors.handlerDistanceSensor.getRange(Unit.kInches) < 5;
    } else {
      if (getState() == CoralHandlerState.GRAB) {
        intakeSim.startIntake();
      } else {
        intakeSim.stopIntake();
      }
      if (getState() == CoralHandlerState.RELEASE && intakeSim.obtainGamePieceFromIntake()) {
        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new ReefscapeCoralOnFly(
                    drive.getSimulatedDriveTrainPose().getTranslation(),
                    new Translation2d(Inches.of(16.0), Inches.of(0)),
                    drive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    drive.getSimulatedDriveTrainPose().getRotation(),
                    elevator.getSimEjectHeight(),
                    MetersPerSecond.of(0.8), // eject speed
                    Radians.of(-Math.PI / 9)));
      }
      hasCoral = intakeSim.getGamePiecesAmount() != 0;
    }
  }

  /** Intake a simulated coral from thin air, like magic */
  public void getSimCoral() {
    if (!intakeSim.addGamePieceToIntake()) {
      System.err.println("You already have a coral, you greedy bastard!");
    }
  }
}
