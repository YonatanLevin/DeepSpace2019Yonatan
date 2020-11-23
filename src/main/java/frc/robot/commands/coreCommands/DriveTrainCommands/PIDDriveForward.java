/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.coreCommands.DriveTrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDDriveForward extends CommandBase {

  private final DriveTrain driveTrain;
  private final double distance;
  private final double tolarance;
  private double endPosition;

  /**
   * Creates a new PIDDriveForward.
   */
  public PIDDriveForward(DriveTrain driveTrain, double distance, double kp, double ki, double kd, double tolarance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.distance = distance;
    this.driveTrain.configPIDSlot(kp, ki, kd, 4);
    this.tolarance = tolarance;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.endPosition = this.driveTrain.getRawLeftPosition() + this.distance;
    this.driveTrain.setPosition(this.endPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveTrain.tankDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.driveTrain.getRawLeftPosition() > this.endPosition - this.tolarance && this.driveTrain.getRawLeftPosition() < this.endPosition + this.tolarance;
  }
}
