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

  public static enum CoralLevel {
    L1,
    L2,
    L3,
    L4
  }

  public AutoRoutine driveOut() {
    AutoRoutine routine = m_factory.newRoutine("driveOut");
    AutoTrajectory driveOut = routine.trajectory("driveOut");

    routine.active().onTrue(Commands.sequence(driveOut.resetOdometry(), driveOut.cmd()));

    return routine;
  }

  public AutoRoutine leftIndia(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("leftIndia" + level);
    AutoTrajectory leftStartToIndia = routine.trajectory("leftStartToIndia");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leftStartToIndia.resetOdometry(),
                leftStartToIndia.cmd(),
                levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine leftIndiaKilo(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("leftIndiaKilo" + level);
    AutoTrajectory leftStartToIndia = routine.trajectory("leftStartToIndia");
    AutoTrajectory indiaToLeftStation = routine.trajectory("indiaToLeftStation");
    AutoTrajectory leftStationToKilo = routine.trajectory("leftStationToKilo");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leftStartToIndia.resetOdometry(),
                leftStartToIndia.cmd(),
                levelSelectorHelper(level),
                indiaToLeftStation.cmd(),
                new GrabCoral(m_coral),
                leftStationToKilo.cmd(),
                levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine leftIndiaKiloLima(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("leftIndiaKiloLima" + level);
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
                levelSelectorHelper(level),
                indiaToLeftStation.cmd(),
                new GrabCoral(m_coral),
                leftStationToKilo.cmd(),
                levelSelectorHelper(level),
                kiloToLeftStation.cmd(),
                new GrabCoral(m_coral),
                leftStationToLima.cmd(),
                levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine rightFoxtrot(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("rightFoxtrot" + level);
    AutoTrajectory rightStartToFoxtrot = routine.trajectory("rightStartToFoxtrot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                rightStartToFoxtrot.resetOdometry(),
                rightStartToFoxtrot.cmd(),
                levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine rightFoxtrotDelta(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("rightFoxtrotDelta" + level);
    AutoTrajectory rightStartToFoxtrot = routine.trajectory("rightStartToFoxtrot");
    AutoTrajectory foxtrotToRightStation = routine.trajectory("foxtrotToRightStation");
    AutoTrajectory rightStationToDelta = routine.trajectory("rightStationToDelta");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                rightStartToFoxtrot.resetOdometry(),
                rightStartToFoxtrot.cmd(),
                levelSelectorHelper(level),
                foxtrotToRightStation.cmd(),
                new GrabCoral(m_coral),
                rightStationToDelta.cmd(),
                levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine rightFoxtrotDeltaCharlie(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("rightFoxtrotDeltaCharlie" + level);
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
                levelSelectorHelper(level),
                foxtrotToRightStation.cmd(),
                new GrabCoral(m_coral),
                rightStationToDelta.cmd(),
                levelSelectorHelper(level),
                deltaToRightStation.cmd(),
                new GrabCoral(m_coral),
                rightStationToCharlie.cmd(),
                levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine middleOwnIndia(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("middleOwnIndia" + level);
    AutoTrajectory middleToIndia = routine.trajectory("middleOwnToIndia");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                middleToIndia.resetOdometry(), middleToIndia.cmd(), levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine middleOwnHotel(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("middleOwnHotel" + level);
    AutoTrajectory middleToHotel = routine.trajectory("middleOwnToHotel");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                middleToHotel.resetOdometry(), middleToHotel.cmd(), levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine middleOpposingFoxtrot(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("middleOpposingFoxtrot" + level);
    AutoTrajectory middleToFoxtrot = routine.trajectory("middleOpposingToFoxtrot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                middleToFoxtrot.resetOdometry(),
                middleToFoxtrot.cmd(),
                levelSelectorHelper(level)));

    return routine;
  }

  public AutoRoutine middleOpposingGamma(CoralLevel level) {
    AutoRoutine routine = m_factory.newRoutine("middleOpposingGamma" + level);
    AutoTrajectory middleToGamma = routine.trajectory("middleOpposingToGamma");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                middleToGamma.resetOdometry(), middleToGamma.cmd(), levelSelectorHelper(level)));

    return routine;
  }

  private Command levelSelectorHelper(CoralLevel level) {
    return switch (level) {
      case L1 -> null;
      case L2 -> upScore2Down();
      case L3 -> upScore3Down();
      case L4 -> upScore4Down();
    };
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
