/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX LFMotor = new TalonSRX(RobotMap.motorLF);
  private TalonSRX LMMotor = new TalonSRX(RobotMap.motorLM);
  private TalonSRX LBMotor = new TalonSRX(RobotMap.motorLB);

  private TalonSRX RFMotor = new TalonSRX(RobotMap.motorRF);
  private TalonSRX RMMotor = new TalonSRX(RobotMap.motorRM);
  private TalonSRX RBMotor = new TalonSRX(RobotMap.motorRB);

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Drive());
  }

  public void driveLeft(double power){
    LFMotor.set(ControlMode.PercentOutput,power);
    LMMotor.set(ControlMode.PercentOutput,power);
    LBMotor.set(ControlMode.PercentOutput,power);
  }

  public void driveRight(double power){
    RFMotor.set(ControlMode.PercentOutput,power);
    RMMotor.set(ControlMode.PercentOutput,power);
    RBMotor.set(ControlMode.PercentOutput,power);
  }

  public void drive(double power){
    driveLeft(power);
    driveRight(power);
  }

  public void turn(double power){
    driveLeft(-power);
    driveRight(power);
  }
}
