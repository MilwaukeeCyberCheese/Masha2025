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
    AutoTrajectory mainTraj = routine.trajectory("Blue Top");

    routine.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));

    return routine;
  }
}
