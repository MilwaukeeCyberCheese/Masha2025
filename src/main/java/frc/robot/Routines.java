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
    * Routine to drive to the processor, drop off the algae, and drive back to the coral station.
    */
  public AutoRoutine blueProcessor() {
    AutoRoutine routine = m_factory.newRoutine("Blue Processor");
    AutoTrajectory mainTraj = routine.trajectory("Blue Processor To Coral Station");

    routine.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));

    return routine;
  }
}
