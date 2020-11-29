/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.coreCommands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class PIDMoveElevator extends CommandBase {

  private final ElevatorSubsystem elevatorSubsystem;
  private final int target;
  private final int tolarance;
  private int pos;//Current position
  private final int kp;
  private final int ki;
  private final int kd;
  private final int kf;

  /**
   * Creates a new PIDMoveElevator.
   */
  public PIDMoveElevator(ElevatorSubsystem elevatorSubsystem, int target, int tolarance, int kp, int ki, int kd, int kf) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.target = target;
    this.tolarance = tolarance;
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.kf = kf;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
}
