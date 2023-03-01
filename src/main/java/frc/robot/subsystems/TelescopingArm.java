// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopingArm extends SubsystemBase {
  public CANSparkMax teleMotor = new CANSparkMax(14, MotorType.kBrushless);
  /** Creates a new TelescopingArm. */
  public TelescopingArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void Update(double rotControlStick) {
    // This method will be called once per scheduler run
    teleMotor.set(new Joystick(1).getRawAxis(5));
  }
}
