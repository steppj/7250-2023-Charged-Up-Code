// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.w3c.dom.views.DocumentView;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotatingArm;

public class RotArm extends CommandBase {
  /** Creates a new RotArm. */
  public RotatingArm Rotation;
  public double stick;
  public RotArm(RotatingArm Rotation, double stick) {
    addRequirements(Rotation);
    // Use addRequirements() here to declare subsystem dependencies.
    this.Rotation = Rotation;
    this.stick = stick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {Rotation.Update(stick);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
