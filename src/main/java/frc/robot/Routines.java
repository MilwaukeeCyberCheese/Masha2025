package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
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

  public AutoRoutine leftIndia4() {
    AutoRoutine routine = m_factory.newRoutine("leftIndia4");
    AutoTrajectory leftStartToIndia = routine.trajectory("leftStartToIndia");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leftStartToIndia.resetOdometry(), leftStartToIndia.cmd(), upScore4Down()));

    return routine;
  }

  public AutoRoutine leftIndia4Kilo4() {
    AutoRoutine routine = m_factory.newRoutine("leftIndia4Kilo4");
    AutoTrajectory leftStartToIndia = routine.trajectory("leftStartToIndia");
    AutoTrajectory indiaToLeftStation = routine.trajectory("indiaToLeftStation");
    AutoTrajectory leftStationToKilo = routine.trajectory("leftStationToKilo");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leftStartToIndia.resetOdometry(),
                leftStartToIndia.cmd(),
                upScore4Down(),
                indiaToLeftStation.cmd(),
                new GrabCoral(m_coral),
                leftStationToKilo.cmd(),
                upScore4Down()));

    return routine;
  }

  public AutoRoutine leftIndia4Kilo4Lima4() {
    AutoRoutine routine = m_factory.newRoutine("leftIndia4Kilo4Lima4");
    AutoTrajectory leftStartToIndia = routine.trajectory("leftStartToIndia");
    AutoTrajectory indiaToLeftStation = routine.trajectory("indiaToLeftStation");
    AutoTrajectory leftStationToKilo = routine.trajectory("leftStationToKilo");
    AutoTrajectory kiloToLeftStation = routine.trajectory("kiloToLeftStation");
    AutoTrajectory leftStationToLima = routine.trajectory("leftStationToLima");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leftStartToIndia.resetOdometry(),
                leftStartToIndia.cmd(),
                upScore4Down(),
                indiaToLeftStation.cmd(),
                new GrabCoral(m_coral),
                leftStationToKilo.cmd(),
                upScore4Down(),
                kiloToLeftStation.cmd(),
                new GrabCoral(m_coral),
                leftStationToLima.cmd(),
                upScore4Down()));

    return routine;
  }

  public AutoRoutine rightFoxtrot4() {
    AutoRoutine routine = m_factory.newRoutine("leftFoxtrot4");
    AutoTrajectory rightStartToFoxtrot = routine.trajectory("rightStartToFoxtrot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                rightStartToFoxtrot.resetOdometry(), rightStartToFoxtrot.cmd(), upScore4Down()));

    return routine;
  }

  public AutoRoutine rightFoxtrot4Delta4() {
    AutoRoutine routine = m_factory.newRoutine("leftFoxtrot4Delta4");
    AutoTrajectory rightStartToFoxtrot = routine.trajectory("rightStartToFoxtrot");
    AutoTrajectory foxtrotToRightStation = routine.trajectory("foxtrotToRightStation");
    AutoTrajectory rightStationToDelta = routine.trajectory("rightStationToDelta");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                rightStartToFoxtrot.resetOdometry(),
                rightStartToFoxtrot.cmd(),
                upScore4Down(),
                foxtrotToRightStation.cmd(),
                new GrabCoral(m_coral),
                rightStationToDelta.cmd(),
                upScore4Down()));

    return routine;
  }

  public AutoRoutine rightFoxtrot4Delta4Charlie4() {
    AutoRoutine routine = m_factory.newRoutine("leftFoxtrot4Delta4Charlie4");
    AutoTrajectory rightStartToFoxtrot = routine.trajectory("rightStartToFoxtrot");
    AutoTrajectory foxtrotToRightStation = routine.trajectory("foxtrotToRightStation");
    AutoTrajectory rightStationToDelta = routine.trajectory("rightStationToDelta");
    AutoTrajectory deltaToRightStation = routine.trajectory("deltaToRightStation");
    AutoTrajectory rightStationToCharlie = routine.trajectory("rightStationToCharlie");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                rightStartToFoxtrot.resetOdometry(),
                rightStartToFoxtrot.cmd(),
                upScore4Down(),
                foxtrotToRightStation.cmd(),
                new GrabCoral(m_coral),
                rightStationToDelta.cmd(),
                upScore4Down(),
                deltaToRightStation.cmd(),
                new GrabCoral(m_coral),
                rightStationToCharlie.cmd(),
                upScore4Down()));

    return routine;
  }

  public AutoRoutine middleOwnIndia4() {
    AutoRoutine routine = m_factory.newRoutine("middleOwnIndia4");
    AutoTrajectory middleToIndia = routine.trajectory("middleOwnToIndia");

    routine
        .active()
        .onTrue(
            Commands.sequence(middleToIndia.resetOdometry(), middleToIndia.cmd(), upScore4Down()));

    return routine;
  }

  public AutoRoutine middleOpposingFoxtrot4() {
    AutoRoutine routine = m_factory.newRoutine("middleOpposingFoxtrot4");
    AutoTrajectory middleToFoxtrot = routine.trajectory("middleOpposingToFoxtrot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                middleToFoxtrot.resetOdometry(), middleToFoxtrot.cmd(), upScore4Down()));

    return routine;
  }

  private Command upScore2Down() {
    return Commands.sequence(
        Commands.runOnce(m_elevator::L2),
        new WaitCommandMilli(200),
        Commands.runOnce(m_coral::release),
        new WaitCommandMilli(400),
        Commands.runOnce(m_coral::inactive),
        Commands.runOnce(m_elevator::L1));
  }

  private Command upScore3Down() {
    return Commands.sequence(
        Commands.runOnce(m_elevator::L3),
        new WaitCommandMilli(200),
        Commands.runOnce(m_coral::release),
        new WaitCommandMilli(400),
        Commands.runOnce(m_coral::inactive),
        Commands.runOnce(m_elevator::L1));
  }

  private Command upScore4Down() {
    return Commands.sequence(
        Commands.runOnce(m_elevator::L4),
        new WaitCommandMilli(200),
        Commands.runOnce(m_coral::release),
        new WaitCommandMilli(400),
        Commands.runOnce(m_coral::inactive),
        Commands.runOnce(m_elevator::L1));
  }
}
