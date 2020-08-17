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

public class Feeder extends SubsystemBase {

  private final WPI_VictorSPX master;
  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    this.master = new WPI_VictorSPX(0);
    this.configMotor();
    this.stop();
  }

  private void configMotor() {
    this.master.setNeutralMode(NeutralMode.Brake);
    this.master.setInverted(InvertType.None);
  }

  public void set(double power) {
    this.master.set(ControlMode.PercentOutput, power);
  }

  public void stop() {
    this.master.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
