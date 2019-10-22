/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.MoveMotor;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX intakeMotor = new VictorSPX(RobotMap.intake_id);
  private AHRS ahrs = new AHRS(Port.kUSB);

  private double gyroBasePitch = 0.0;
  private double gyroBaseRoll = 0.0;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveMotor());
  }

  public void setIntake(double speed){
    intakeMotor.set(ControlMode.PercentOutput,speed);
  } 

  public void setIntakeVelocity(double vel){
    intakeMotor.set(ControlMode.Position,vel);
  }

  public String getGyro(){
    double pitch = ahrs.getPitch() - gyroBasePitch;
    double roll = ahrs.getRoll() - gyroBaseRoll;
    double yaw = ahrs.getYaw();
    return String.format("Pitch: %f, Roll: %f, Yaw: %f%n",pitch,roll,yaw);

  }

  public double[] getGyroValues(){
    double[] values = new double[3];
    double pitch = ahrs.getPitch() - gyroBasePitch;
    double roll = ahrs.getRoll() - gyroBaseRoll;
    double yaw = ahrs.getYaw();
    values[0] = pitch;
    values[1] = roll;
    values[2] = yaw;
    return values;
    
  }

  public void resetGyro(){
    // Sets the base pitch and roll to the current pitch and roll so it can read 0
    gyroBasePitch = ahrs.getPitch();
    gyroBaseRoll = ahrs.getRoll();

    ahrs.reset();
  }
}
