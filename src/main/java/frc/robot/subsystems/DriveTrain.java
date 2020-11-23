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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import poroslib.subsystems.DiffDrivetrain;

public class DriveTrain extends DiffDrivetrain {

  private WPI_TalonSRX masterLeft;
  private WPI_TalonSRX masterRight;
  private WPI_TalonSRX middleLeft;
  private WPI_TalonSRX middleRight;
  private WPI_VictorSPX rearLeft;
  private WPI_VictorSPX rearRight;

  private final int pidSlot = 4;

  private AHRS navx;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    super(new WPI_TalonSRX(Constants.kForwardLeftPort), new WPI_TalonSRX(Constants.kForwardRightPort));
    this.configMotors();
  }

  private void configMotors() {
    
    this.createMotors();
    this.configFollowers();
    this.setNeutralModes();
    this.setInvertions();
    this.setSensors();

  }

  private void createMotors() {
    this.masterLeft = (WPI_TalonSRX)this.leftController;
    this.masterRight = (WPI_TalonSRX)this.rightController;
    this.middleLeft = new WPI_TalonSRX(Constants.kMiddleLeftPort);
    this.middleRight = new WPI_TalonSRX(Constants.kMiddleRightPort);
    this.rearLeft = new WPI_VictorSPX(Constants.kRearLeftPort);
    this.rearRight = new WPI_VictorSPX(Constants.kRearRightPort);
  }

  private void configFollowers() {
    this.middleLeft.follow(masterLeft);
    this.rearLeft.follow(masterLeft);
    this.middleRight.follow(masterRight);
    this.rearRight.follow(masterRight);
  }

  private void setNeutralModes() {
    NeutralMode nm = NeutralMode.Brake;

    this.masterLeft.setNeutralMode(nm);
    this.masterRight.setNeutralMode(nm);
    this.middleLeft.setNeutralMode(nm);
    this.middleRight.setNeutralMode(nm);
    this.rearLeft.setNeutralMode(nm);
    this.rearRight.setNeutralMode(nm);
  }

  private void setInvertions() {
    this.masterLeft.setInverted(InvertType.None);
    this.masterRight.setInverted(InvertType.None);
    this.middleLeft.setInverted(InvertType.FollowMaster);
    this.middleRight.setInverted(InvertType.FollowMaster);
    this.rearLeft.setInverted(InvertType.FollowMaster);
    this.rearRight.setInverted(InvertType.FollowMaster);
  }

  private void setSensors(){
    this.masterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    this.masterRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    this.navx = new AHRS(SPI.Port.kMXP);
  }

  public void configPIDSlot(double kp, double ki, double kd, int slot){
    this.masterLeft.config_kP(slot, kp);
    this.masterLeft.config_kI(slot, ki);
    this.masterLeft.config_kD(slot, kd);

    this.masterRight.config_kP(slot, kp);
    this.masterRight.config_kI(slot, ki);
    this.masterRight.config_kD(slot, kd);
  }

  public void setPosition(double pos){
    this.masterLeft.set(ControlMode.Position, pos);
    this.masterRight.set(ControlMode.Position, pos);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public int getRawLeftPosition() {
    // TODO Auto-generated method stub
    return this.masterLeft.getSelectedSensorPosition();
  }

  @Override
  public int getRawRightPosition() {
    // TODO Auto-generated method stub
    return this.masterRight.getSelectedSensorPosition();
  }

  @Override
  public double getHeading() {
    // TODO Auto-generated method stub
    return this.navx.getYaw();
  }

  public int getPIDSlot(){
    return this.pidSlot;
  }
}
