// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** This is a filter class for the custom buttons on the driver station */
public class FilteredButton {
  private Joystick controller;

  /**
   * Filter for the custom buttons and switches
   *
   * @param port the port the controller is connected to on the driver station
   */
  public FilteredButton(int port) {
    this.controller = new Joystick(port);
  }

  /**
   * Returns if the L1 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getA1() {
    return new Trigger(() -> controller.getRawButton(1));
  }

  /**
   * Returns if the L2 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getA2() {
    return new Trigger(() -> controller.getRawButton(2));
  }

  /**
   * Returns if the L3 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getA3() {
    return new Trigger(() -> controller.getRawButton(3));
  }

  /**
   * Returns if the L1 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getB1() {
    return new Trigger(() -> controller.getRawButton(4));
  }

  /**
   * Returns if the L2 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getB2() {
    return new Trigger(() -> controller.getRawButton(5));
  }

  /**
   * Returns if the L3 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getB3() {
    return new Trigger(() -> controller.getRawButton(6));
  }

  /**
   * Returns if the L1 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getC1() {
    return new Trigger(() -> controller.getRawButton(7));
  }

  /**
   * Returns if the L2 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getC2() {
    return new Trigger(() -> controller.getRawButton(8));
  }

  /**
   * Returns if the L3 button has been pressed
   *
   * @return Trigger
   */
  public Trigger getC3() {
    return new Trigger(() -> controller.getRawButton(9));
  }
}
