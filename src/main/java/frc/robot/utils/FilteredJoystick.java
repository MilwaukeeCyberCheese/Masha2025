package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class FilteredJoystick {
  private final Joystick joystick;

  /**
   * Filter class for the joysticks
   *
   * @param port the port the joystick is plugged into
   */
  public FilteredJoystick(int port) {
    joystick = new Joystick(port);
  }

  public Joystick getJoystick() {
    return joystick;
  }

  /**
   * Returns the x-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getX(double deadzone) {
    return MathUtil.applyDeadband(joystick.getX(), deadzone);
  }

  /**
   * Returns the y-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getY(double deadzone) {
    return MathUtil.applyDeadband(joystick.getY(), deadzone) * -1;
  }

  /**
   * Returns the z-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getZ(double deadzone) {
    return MathUtil.applyDeadband(joystick.getZ(), deadzone);
  }

  /**
   * Returns the throttle-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getThrottle(double deadzone) {
    return ((MathUtil.applyDeadband(joystick.getThrottle(), deadzone) * -1) + 1) / 2;
  }

  /**
   * Returns the twist-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getTwist(double deadzone) {
    return MathUtil.applyDeadband(joystick.getTwist(), deadzone);
  }

  /** Returns the x-value of the joystick */
  public double getX() {
    return this.getX(Constants.IOConstants.kDriveDeadband);
  }

  /**
   * Returns the y-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getY() {
    return this.getY(Constants.IOConstants.kDriveDeadband);
  }

  /**
   * Returns the z-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getZ() {
    return this.getZ(Constants.IOConstants.kDriveDeadband);
  }

  /**
   * Returns the throttle-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getThrottle() {
    return this.getThrottle(Constants.IOConstants.kDriveDeadband);
  }

  /**
   * Returns the twist-value of the joystick
   *
   * @param deadzone zone in which no value is returned
   */
  public double getTwist() {
    return this.getTwist(0.2);
  }

  /**
   * Returns whether or not the trigger is pressed
   *
   * @return Trigger
   */
  public Trigger getTriggerActive() {
    return new Trigger(() -> joystick.getRawButton(1));
  }

  /**
   * Returns if any POVButton is pressed or not
   *
   * @return Trigger
   */
  public Trigger getPOVPressed() {
    return new Trigger(() -> joystick.getPOV() != -1);
  }

  /**
   * Returns the state of the POV
   *
   * @return the degree of the pov, -1 if not pressed
   */
  public int getPovState() {
    return joystick.getPOV();
  }

  public Trigger getButtonTwo() {
    return new Trigger(() -> joystick.getRawButton(2));
  }

  public Trigger getButtonThree() {
    return new Trigger(() -> joystick.getRawButton(3));
  }

  public Trigger getButtonFour() {
    return new Trigger(() -> joystick.getRawButton(4));
  }

  public Trigger getButtonFive() {
    return new Trigger(() -> joystick.getRawButton(5));
  }

  public Trigger getButtonSix() {
    return new Trigger(() -> joystick.getRawButton(6));
  }

  public Trigger getButtonSeven() {
    return new Trigger(() -> joystick.getRawButton(7));
  }

  public Trigger getButtonEight() {
    return new Trigger(() -> joystick.getRawButton(8));
  }

  public Trigger getButtonNine() {
    return new Trigger(() -> joystick.getRawButton(9));
  }

  public Trigger getButtonTen() {
    return new Trigger(() -> joystick.getRawButton(10));
  }

  public Trigger getButtonEleven() {
    return new Trigger(() -> joystick.getRawButton(11));
  }

  public Trigger getButtonTwelve() {
    return new Trigger(() -> joystick.getRawButton(12));
  }
}
