/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.coreCommands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class DebagPIDMoveElevator extends CommandBase {

  private final ElevatorSubsystem elevatorSubsystem;
  private final int tolarance;
  private int target;
  private int pos;//Current position
  private double kp;
  private double ki;
  private double kd;
  private double kf;

  /**
   * Creates a new PIDMoveElevator.
   */
  public DebagPIDMoveElevator(ElevatorSubsystem elevatorSubsystem, int tolarance) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.tolarance = tolarance;

    addRequirements(elevatorSubsystem);
    
    this.initDashboardValues();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.updateDashboardValues();
    this.elevatorSubsystem.configPIDSlot(kp, ki, kd, kf, elevatorSubsystem.getPIDSlotID());
    this.elevatorSubsystem.setPosition(this.target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.pos = this.elevatorSubsystem.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.pos >= this.target - this.tolarance && this.pos <= this.target + this.tolarance;
  }

  private void initDashboardValues(){
    SmartDashboard.putNumber("kp", 0);
    SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kd", 0);
    SmartDashboard.putNumber("kf", 0);
    SmartDashboard.putNumber("target", 0);
  }

  private void updateDashboardValues(){
    this.kp = SmartDashboard.getNumber("kp", 0);
    this.ki = SmartDashboard.getNumber("ki", 0);
    this.kd = SmartDashboard.getNumber("kd", 0);
    this.kf = SmartDashboard.getNumber("kf", 0);
    this.target = (int)SmartDashboard.getNumber("target", 0);
  }
}
