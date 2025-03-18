package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.FilteredJoystick;

public class Controllers {

    // Driver joysticks
    public final FilteredJoystick leftJoystick =
            new FilteredJoystick(Constants.IOConstants.kLeftJoystickPort);
    public final FilteredJoystick rightJoystick =
            new FilteredJoystick(Constants.IOConstants.kRightJoystickPort);

    // Operator controller
    public final CommandXboxController controller =
            new CommandXboxController(Constants.IOConstants.kControllerPort);

    private ControllerState controllerState = Robot.isSimulation() ? ControllerState.XBOX : ControllerState.JOYSTICKS;

    public Controllers() {
        SmartDashboard.putData("Controllers/Xbox", this.controller.getHID());

        this.switchController(this.controllerState);
    }

    private boolean isCurrentConnected() {
        return switch (this.controllerState) {
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
    }

    public void updateControllerConnections() {
        if (!this.isCurrentConnected()) {
            if (this.controller.isConnected()) this.switchController(ControllerState.XBOX);
            else if (this.isAnyJoystickConnected()) this.switchController(ControllerState.JOYSTICKS);
        }
    }

    public enum ControllerState {
        XBOX,
        JOYSTICKS,
    }
}
