package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class GenericAutoCommand {

  public static Command genericAutoCommand() {
    try {
      // Loading PathPlanner path using name in GUI
      // PathPlannerPath path = PathPlannerPath.fromPathFile("path");
      // Load the Choreo Path you want to use
      PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory("Blue Top");
      // Create the path following command using AutoBuilder
      return AutoBuilder.followPath(choreoPath);
    } catch (Exception e) {
      DriverStation.reportError(
          "Couldn't load Choreo path to PathPlanner: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
