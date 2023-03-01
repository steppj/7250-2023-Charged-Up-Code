// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// Using "import static an.enum.or.constants.inner.class.*;" helps reduce verbosity
// this replaces "DoubleSolenoid.Value.kForward" with just kForward
// further reading is available at https://www.geeksforgeeks.org/static-import-java/
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClampArm extends SubsystemBase {
  DoubleSolenoid exampleDoublePCM = new DoubleSolenoid(0,PneumaticsModuleType.CTREPCM, 2, 3);
  Boolean On = false;
  /** Creates a new ClampArm. */
  public ClampArm() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void MoveArm() //kForward,kReverse,KOff, mess with these to make it work
  {
    System.out.println(On);
    if(!On)
    {
      exampleDoublePCM.set(kForward);
      On = true;
    }else if(On)
    {
      exampleDoublePCM.set(kReverse);
      On = false;
    }
  }
}
