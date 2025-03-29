package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.GrabCoral;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class Routines {
  private final AutoFactory m_factory;
  private final ElevatorSubsystem m_elevator;
  private final CoralHandlerSubsystem m_coral;

  public Routines(AutoFactory factory, ElevatorSubsystem elevator, CoralHandlerSubsystem coral) {
    m_factory = factory;
    m_elevator = elevator;
    m_coral = coral;
  }

  public AutoRoutine driveOut() {
    AutoRoutine routine = m_factory.newRoutine("driveOut");
    AutoTrajectory driveOut = routine.trajectory("driveOut");

    routine.active().onTrue(Commands.sequence(driveOut.resetOdometry(), driveOut.cmd()));

    return routine;
  }

  public AutoRoutine scoreIndia4() {
    AutoRoutine routine = m_factory.newRoutine("scoreIndia4");
    AutoTrajectory leftStartToIndia = routine.trajectory("leftStartToIndia");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leftStartToIndia.resetOdometry(),
                leftStartToIndia.cmd(),
                Commands.runOnce(m_elevator::L4),
                new WaitCommandMilli(200),
                Commands.runOnce(m_coral::release),
                new WaitCommandMilli(400),
                Commands.runOnce(m_coral::inactive),
                Commands.runOnce(m_elevator::L1)));

    return routine;
  }

  public AutoRoutine scoreIndia4Kilo4() {
    AutoRoutine routine = m_factory.newRoutine("scoreIndia4Kilo4");
    AutoTrajectory leftStartToIndia = routine.trajectory("leftStartToIndia");
    AutoTrajectory indiaToLeftStation = routine.trajectory("indiaToLeftStation");
    AutoTrajectory leftStationToKilo = routine.trajectory("leftStationToKilo");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leftStartToIndia.resetOdometry(),
                leftStartToIndia.cmd(),
                Commands.runOnce(m_elevator::L4),
                new WaitCommandMilli(200),
                Commands.runOnce(m_coral::release),
                new WaitCommandMilli(400),
                Commands.runOnce(m_coral::inactive),
                Commands.runOnce(m_elevator::L1),
                indiaToLeftStation.cmd(),
                new GrabCoral(m_coral),
                leftStationToKilo.cmd(),
                Commands.runOnce(m_elevator::L4),
                new WaitCommandMilli(200),
                Commands.runOnce(m_coral::release),
                new WaitCommandMilli(400),
                Commands.runOnce(m_coral::inactive),
                Commands.runOnce(m_elevator::L1)));

    return routine;
  }

  public AutoRoutine scoreFoxtrot4() {
    AutoRoutine routine = m_factory.newRoutine("scoreFoxtrot4");
    AutoTrajectory rightStartToFoxtrot = routine.trajectory("rightStartToFoxtrot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                rightStartToFoxtrot.resetOdometry(),
                rightStartToFoxtrot.cmd(),
                Commands.runOnce(m_elevator::L4),
                new WaitCommandMilli(200),
                Commands.runOnce(m_coral::release),
                new WaitCommandMilli(400),
                Commands.runOnce(m_coral::inactive),
                Commands.runOnce(m_elevator::L1)));

    return routine;
  }

  public AutoRoutine scoreFoxtrot4Delta4() {
    AutoRoutine routine = m_factory.newRoutine("scoreFoxtrot4Delta4");
    AutoTrajectory rightStartToFoxtrot = routine.trajectory("rightStartToFoxtrot");
    AutoTrajectory foxtrotToRightStation = routine.trajectory("foxtrotToRightStation");
    AutoTrajectory rightStationToDelta = routine.trajectory("rightStationToDelta");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                rightStartToFoxtrot.resetOdometry(),
                rightStartToFoxtrot.cmd(),
                Commands.runOnce(m_elevator::L4),
                new WaitCommandMilli(200),
                Commands.runOnce(m_coral::release),
                new WaitCommandMilli(400),
                Commands.runOnce(m_coral::inactive),
                Commands.runOnce(m_elevator::L1),
                foxtrotToRightStation.cmd(),
                new GrabCoral(m_coral),
                rightStationToDelta.cmd(),
                Commands.runOnce(m_elevator::L4),
                new WaitCommandMilli(200),
                Commands.runOnce(m_coral::release),
                new WaitCommandMilli(400),
                Commands.runOnce(m_coral::inactive),
                Commands.runOnce(m_elevator::L1)));

    return routine;
  }
}
