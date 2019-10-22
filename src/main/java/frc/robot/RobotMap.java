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

  public static final int intake_id = 35;
  public static final int joystick_id = 0;

  public static final int Left_Stick_Y = 1;
  public static final int Right_Stick_Y = 2;
  public static final int Button_A = 1;
  public static final int Button_B = 2;
  public static final int RT = 6;


  // These values are currently unmapped
  public static final int motorLF = 0;
  public static final int motorLMaster = 0;
  public static final int motorLB = 0;
  public static final int motorRF = 0;
  public static final int motorRMaster = 0;
  public static final int motorRB = 0;
}
