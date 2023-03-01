// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClampArm;

public class ClampPads extends CommandBase {
  private ClampArm clamp;
  /** Creates a new ClampPads. */
  public ClampPads(ClampArm clamp) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clamp = clamp;
    addRequirements(clamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {clamp.MoveArm();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
