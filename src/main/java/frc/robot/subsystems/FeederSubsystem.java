/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  /**
   * Creates a new FeederSubsystem.
   */

  private final WPI_VictorSPX master;
  
  public FeederSubsystem() {
    this.master = new WPI_VictorSPX(Constants.kFeederPort);
    this.configMotor();
    this.stop();
  }

  public void configMotor(){
    this.master.setNeutralMode(NeutralMode.Brake);
    this.master.setInverted(InvertType.None);
  }

  public void set(double power) {
    this.master.set(ControlMode.PercentOutput,power);
  }

  public void stop() {
    this.master.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
