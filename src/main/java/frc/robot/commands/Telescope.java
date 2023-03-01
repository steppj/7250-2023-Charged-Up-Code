// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArm;

public class Telescope extends CommandBase {
  private TelescopingArm tele;
  private double stick;
  /** Creates a new Telescope. */
  public Telescope(TelescopingArm tele, double stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tele = tele;
    this.stick = stick;
    addRequirements(tele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {tele.Update(0);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
