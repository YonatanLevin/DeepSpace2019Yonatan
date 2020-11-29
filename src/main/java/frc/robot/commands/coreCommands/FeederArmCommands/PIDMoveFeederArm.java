/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.coreCommands.FeederArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederArmSubsystem;

public class PIDMoveFeederArm extends CommandBase {

  private final FeederArmSubsystem feederArmSubsystem;
  private final int target;
  private final int tolerance;
  private final int kp;
  private final int ki;
  private final int kd;
  private int pos;// The current position

  /**
   * Creates a new PIDMoveFeederArm.
   */
  public PIDMoveFeederArm(FeederArmSubsystem feederArmSubsystem, int target, int tolerance, int kp, int ki, int kd) {
    addRequirements(feederArmSubsystem);

    this.feederArmSubsystem = feederArmSubsystem;
    this.target = target;
    this.tolerance = tolerance;
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.pos = this.feederArmSubsystem.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feederArmSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.pos <= this.target + this.tolerance && this.pos >= this.target - this.tolerance;
  }
}
