/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.coreCommands.FeederArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederArmSubsystem;

public class MagicMoveFeederArm extends CommandBase {
  private final FeederArmSubsystem feederArmSubsystem;
  private final int target;

  /**
   * Creates a new MagicMoveFeederArm.
   */
  public MagicMoveFeederArm(FeederArmSubsystem feederArmSubsystem, int target) {
    this.feederArmSubsystem = feederArmSubsystem;
    this.target = target;
    addRequirements(feederArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.feederArmSubsystem.setMagic(this.target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feederArmSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}