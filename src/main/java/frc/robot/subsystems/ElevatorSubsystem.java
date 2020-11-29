/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /**
   * Creates a new ElevatorSubsystem.
   */

  private final WPI_TalonSRX master;

  private final int pidSlotID;
  
  public ElevatorSubsystem() {
    this.pidSlotID = 3;
    this.master = new WPI_TalonSRX(Constants.kElevatorPort);
    this.configMotor();
    this.stop();
  }

  public void configMotor(){
    this.master.setNeutralMode(NeutralMode.Brake);
    this.master.setInverted(InvertType.None);
    this.master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    this.master.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    this.master.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, Constants.kMiddleLeftPort);
    }
  
  public void set(double power) {
    this.master.set(ControlMode.PercentOutput,power);
  }

  public void stop() {
    this.master.stopMotor();
  }

  public int getPosition() {
    return this.master.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(this.master.getSensorCollection().is)

    SmartDashboard.putNumber("Elevator position", getPosition());
  }

  public int getPIDSlotID(){
    return this.pidSlotID;
  }

  public void setPosition(int target){
    this.master.selectProfileSlot(this.pidSlotID, 0);
    this.master.set(ControlMode.Position, target);
  }

  public void configPIDSlot(double kp, double ki, double kd, double kf, int slot){
    this.master.config_kP(slot, kp);
    this.master.config_kI(slot, ki);
    this.master.config_kD(slot, kd);
    this.master.config_kF(slot, kf);
  }
}
