package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class Routines {
  private final AutoFactory m_factory;

  public Routines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine test() {
    AutoRoutine routine = m_factory.newRoutine("Test");
    AutoTrajectory mainTraj = routine.trajectory("Test");

    routine.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));

    return routine;
  }

  /**
   * Routine to drive to the Processor, drop off the Algae, and drive back to the starting point.
   */
  public AutoRoutine blueProcessor() {
    AutoRoutine routine = m_factory.newRoutine("Blue Processor");
    AutoTrajectory mainTraj = routine.trajectory("Blue To Processor");

    routine.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));

    return routine;
  }

  /** Routine to drive to the Coral Station. */
  public AutoRoutine blueCoralStation() {
    AutoRoutine routine = m_factory.newRoutine("Blue Coral Station");
    AutoTrajectory mainTraj = routine.trajectory("Blue To Coral Station");

    routine.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));

    return routine;
  }

  /** Routine to drive to the Coral Station. */
  public AutoRoutine blueCoralToReefK() {
    AutoRoutine routine = m_factory.newRoutine("Blue Coral Station To Reef K");
    AutoTrajectory mainTraj = routine.trajectory("Blue Coral Station To Reef K");

    routine.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));

    return routine;
  }

  /** Routine to drive to the Coral Station. */
  public AutoRoutine blueTestFull() {
    AutoRoutine routine = m_factory.newRoutine("Blue Test Full");
    AutoTrajectory processorTraj = routine.trajectory("Blue To Processor");
    AutoTrajectory coralStationTraj = routine.trajectory("Blue To Coral Station");
    AutoTrajectory reefKTraj = routine.trajectory("Blue Coral Station To Reef K");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                processorTraj.resetOdometry(),
                processorTraj.cmd(),
                coralStationTraj.cmd(),
                reefKTraj.cmd()));

    return routine;
  }
}
