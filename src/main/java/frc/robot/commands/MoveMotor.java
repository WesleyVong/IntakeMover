/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class MoveMotor extends Command {

  double scale = 0.1;
  boolean gyroOut = false;

  public MoveMotor() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    


    double leftStickY = Robot.m_oi.getControllerRawAxis(RobotMap.Left_Stick_Y);
    boolean ButtonA = Robot.m_oi.getControllerButton(RobotMap.Button_A);
    boolean ButtonB = Robot.m_oi.getControllerButton(RobotMap.Button_B);
    boolean RT = Robot.m_oi.getControllerButton(RobotMap.RT);
    
    if (ButtonA){
      scale = 0.1;
    }
    else {
      scale = 0.25;
    }
    if (ButtonB && !gyroOut){
      gyroOut = true;
      System.out.println(Robot.intake.getGyro());
    } else if (!ButtonB) {
      gyroOut = false;
    }
    if (RT) {
      Robot.intake.resetGyro();
    }
    Robot.intake.setIntake(leftStickY * scale);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intake.setIntake(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
