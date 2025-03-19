package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.FilteredJoystick;
import java.util.List;

public class Controllers {

  private static final ControllerState DEFAULT_CONTROLLER_STATE =
      Robot.isSimulation() ? ControllerState.XBOX : ControllerState.JOYSTICKS;

  private final Alert simulationJoysticksAlert =
      new Alert("Joysticks are connected in simulation", Alert.AlertType.kWarning);
  private final Alert realXboxAlert =
      new Alert("Xbox controller is connected to physical robot", Alert.AlertType.kWarning);

  // Driver joysticks
  public final FilteredJoystick leftJoystick =
      new FilteredJoystick(Constants.IOConstants.kLeftJoystickPort);
  public final FilteredJoystick rightJoystick =
      new FilteredJoystick(Constants.IOConstants.kRightJoystickPort);

  // Operator controller
  public final CommandXboxController controller =
      new CommandXboxController(Constants.IOConstants.kControllerPort);

  private ControllerState controllerState = DEFAULT_CONTROLLER_STATE;

  private double lastControllerConsoleWarning = 0.;

  public Controllers() {
    SmartDashboard.putData("Controllers/Xbox", this.controller.getHID());

    this.switchController(this.controllerState);

    this.controller
        .start()
        .onTrue(Commands.runOnce(() -> this.switchController(ControllerState.XBOX)));
    this.leftJoystick
        .getButtonNine()
        .onTrue(Commands.runOnce(() -> this.switchController(ControllerState.JOYSTICKS)));
    this.rightJoystick
        .getButtonNine()
        .onTrue(Commands.runOnce(() -> this.switchController(ControllerState.JOYSTICKS)));
  }

  private boolean isConnected(ControllerState state) {
    return switch (state) {
      case XBOX -> this.isXboxConnected();
      case JOYSTICKS -> this.isAnyJoystickConnected();
    };
  }

  private boolean isXboxConnected() {
    return this.controller.isConnected();
  }

  private boolean isAnyJoystickConnected() {
    return this.leftJoystick.getJoystick().isConnected()
        || this.rightJoystick.getJoystick().isConnected();
  }

  public boolean useXboxController() {
    return this.controllerState != ControllerState.JOYSTICKS;
  }

  public double getControllerX() {
    if (this.useXboxController()) {
      return controller.getLeftX();
    } else {
      return leftJoystick.getX();
    }
  }

  public double getControllerY() {
    if (this.useXboxController()) {
      return controller.getLeftY();
    } else {
      return leftJoystick.getY();
    }
  }

  public double getControllerRotation() {
    if (this.useXboxController()) {
      return -controller.getRightX();
    } else {
      return rightJoystick.getX();
    }
  }

  public boolean getControllerSlow() {
    if (this.useXboxController()) {
      return controller.rightBumper().getAsBoolean();
    } else {
      return rightJoystick.getButtonTwo().getAsBoolean();
    }
  }

  public double getControllerThrottle() {
    if (this.useXboxController()) {
      return 1.0;
    } else {
      return leftJoystick.getThrottle();
    }
  }

  public void switchController(ControllerState newState) {
    this.controllerState = newState;
    System.out.println("Controller: " + this.controllerState);
    SmartDashboard.putString("Controller", this.controllerState.toString());

    this.simulationJoysticksAlert.set(
        Robot.isSimulation() && this.controllerState == ControllerState.JOYSTICKS);
    // TODO: Maybe only with fms?
    this.realXboxAlert.set(Robot.isReal() && this.controllerState == ControllerState.XBOX);
  }

  public void updateControllerConnections() {
    final var time = Timer.getTimestamp();
    if (this.realXboxAlert.get() && this.lastControllerConsoleWarning < time - 15.) {
      this.lastControllerConsoleWarning = time;
      System.err.println(
          "Oops! Looks like you have the Xbox controller selected for drive on a physical robot.");
    }

    if (!this.isConnected(this.controllerState)) {
      // Try the default first, then try everything else.
      if (this.isConnected(DEFAULT_CONTROLLER_STATE)) {
        this.switchController(DEFAULT_CONTROLLER_STATE);
      } else {
        for (final var controller : ControllerState.VALUES) {
          if (this.isConnected(controller)) {
            this.switchController(controller);
            break;
          }
        }
      }
    }
  }

  public enum ControllerState {
    XBOX,
    JOYSTICKS,
    ;

    public static final List<ControllerState> VALUES = List.of(values());
  }
}
