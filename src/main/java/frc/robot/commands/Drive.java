/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Drive extends Command {
  double leftInput = 0.0;
  double rightInput = 0.0;
  boolean isArcade = false;
  boolean modeChange = false;

  public Drive() {

    
    requires(Robot.intake);
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double leftStickX = Robot.m_oi.getControllerRawAxis(RobotMap.Left_Stick_X);
    double leftStickY = Robot.m_oi.getControllerRawAxis(RobotMap.Left_Stick_Y);
    double rightStickY = Robot.m_oi.getControllerRawAxis(RobotMap.Right_Stick_Y);
    boolean ButtonA = Robot.m_oi.getControllerButton(RobotMap.Button_A);
    boolean ButtonB = Robot.m_oi.getControllerButton(RobotMap.Button_B);
    boolean RT = Robot.m_oi.getControllerButton(RobotMap.RT);

    if (RT && !modeChange){
      modeChange = true;
      isArcade = !isArcade;
    } else if (!RT && modeChange){
      modeChange = false;
    }
    if (ButtonA){
      Robot.intake.setIntake(0.25);
    }
    else if (ButtonB){
      Robot.intake.setIntake(-0.25);
    }
    else {
      Robot.intake.setIntake(0);
    }

    // Tank
    if (!isArcade){
      leftInput = leftStickY * Math.abs(leftStickY);
      rightInput = rightStickY * Math.abs(rightStickY);
      Robot.driveTrain.driveLeft(leftInput);
      Robot.driveTrain.driveRight(rightInput);
    }
    else if (isArcade){
      leftInput = leftStickX * Math.abs(leftStickX);
      rightInput = rightStickY * Math.abs(rightStickY);
      Robot.driveTrain.arcade(leftInput,rightInput);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
