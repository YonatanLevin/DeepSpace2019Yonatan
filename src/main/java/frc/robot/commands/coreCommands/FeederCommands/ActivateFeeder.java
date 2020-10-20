/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.coreCommands.FeederCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class ActivateFeeder extends CommandBase {
  /**
   * Creates a new ActivateFeeder.
   */

  private final FeederSubsystem feeder;
  private final double power;

  public ActivateFeeder(FeederSubsystem feeder, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeder = feeder;
    this.power = power;
    addRequirements(feeder);
  }

  /**
   * ActivateFeeder with timeout
   * @param duration in seconds
   */
  public ActivateFeeder(FeederSubsystem feeder, double power, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeder = feeder;
    this.power = power;
    addRequirements(feeder);
    this.withTimeout(duration);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.feeder.set(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
