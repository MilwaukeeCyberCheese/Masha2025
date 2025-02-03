// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

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
   * @return boolean
   */
  public boolean getL1() {
    return controller.getRawButton(1);
  }

  /**
   * Returns if the L2 button has been pressed
   *
   * @return boolean
   */
  public boolean getL2() {
    return controller.getRawButton(2);
  }

  /**
   * Returns if the L3 button has been pressed
   *
   * @return boolean
   */
  public boolean getL3() {
    return controller.getRawButton(3);
  }

  /**
   * Returns if the L4 button has been pressed
   *
   * @return boolean
   */
  public boolean getL4() {
    return controller.getRawButton(4);
  }

  /**
   * returns if the chute switch is on or off
   *
   * @return boolean
   */
  public boolean getChuteSwitch() {
    return (controller.isConnected()) ? controller.getRawButton(5) : false;
  }

  /**
   * returns if the bottom switch is on or off
   *
   * @return boolean
   */
  public boolean getCoralSwitch() {
    return controller.getRawButton(6);
  }
}
