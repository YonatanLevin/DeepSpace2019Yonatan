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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederArmSubsystem extends SubsystemBase {
  /**
   * Creates a new FeederSubsystem.
   */

  private final WPI_TalonSRX master;
  private final int pidSlotID = 3;
  private final int magicSlotID = 0;

  private final int kMaxAcceleration = 0;
  private final int kCruiseVelocity = 0;
  private final int kMagicP = 0;
  private final int kMagicI = 0;
  private final int kMagicD = 0;
  private final int kMagicF = 1023 / kCruiseVelocity;
  
  public FeederArmSubsystem() {
    this.master = new WPI_TalonSRX(Constants.kFeederArmPort);
    this.configMotor();
    this.stop();
  }

  public void configMotor(){
    this.master.setNeutralMode(NeutralMode.Brake);
    this.master.setInverted(InvertType.None);
    this.master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    this.master.configMotionAcceleration(this.kMaxAcceleration);
    this.master.configMotionCruiseVelocity(this.kCruiseVelocity);
    this.configSlot(this.kMagicP, this.kMagicI, this.kMagicD, this.kMagicF, this.magicSlotID);
  }

  public int getPosition(){
    return this.master.getSelectedSensorPosition();
  }

  public void set(double power) {
    this.master.set(ControlMode.PercentOutput,power);
  }

  public void stop() {
    this.master.stopMotor();
  }

  public int getPidSlotID() {
    return this.pidSlotID;
  }

  public void configSlot(double kp, double ki, double kd, double kf, int slot){
    this.master.config_kP(slot, kp);
    this.master.config_kI(slot, ki);
    this.master.config_kD(slot, kd);
    this.master.config_kF(slot, kf);
  }

  public void setPosition(int pos){
    this.master.selectProfileSlot(this.pidSlotID, 0);
    this.master.set(ControlMode.Position, pos);
  }

  public void setMagic(int pos){
    this.master.selectProfileSlot(this.magicSlotID, 0);
    this.master.set(ControlMode.MotionMagic, pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
