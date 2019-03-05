/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // OI AXES
  public static final int LEFT_JOYSTICK_X = 0;
  public static final int LEFT_JOYSTICK_Y = 1;
  public static final int RIGHT_JOYSTICK_X = 4;
  public static final int RIGHT_JOYSTICK_Y = 5;
  public static final int LEFT_TRIGGER = 2;
  public static final int RIGHT_TRIGGER = 3;

  // OI BUTTON PORTS
  public static final int A = 1;
  public static final int B = 2;
  public static final int X = 3;
  public static final int Y = 4;
  public static final int LEFT_BUMPER = 5;
  public static final int RIGHT_BUMPER = 6;
  public static final int BACK = 7;
  public static final int START = 8;
  public static final int LEFT_JOYSTICK = 9;
  public static final int RIGHT_JOYSTICK = 10;
}
