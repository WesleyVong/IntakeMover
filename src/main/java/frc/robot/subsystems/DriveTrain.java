/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  VictorSPX LFMotor = new VictorSPX(RobotMap.motorLF);
  TalonSRX LMaster = new TalonSRX(RobotMap.motorLMaster);
  VictorSPX LBMotor = new VictorSPX(RobotMap.motorLB);

  VictorSPX RFMotor = new VictorSPX(RobotMap.motorRF);
  TalonSRX RMaster = new TalonSRX(RobotMap.motorRMaster);
  VictorSPX RBMotor = new VictorSPX(RobotMap.motorRB);

  RFMotor.setInverted(true);

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Drive());
  }

  public void driveLeft(double power){
    LFMotor.set(ControlMode.PercentOutput,power);
    LMaster.set(ControlMode.PercentOutput,power);
    LBMotor.set(ControlMode.PercentOutput,power);
  }

  public void driveRight(double power){
    RFMotor.set(ControlMode.PercentOutput,power);
    RMaster.set(ControlMode.PercentOutput,power);
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
