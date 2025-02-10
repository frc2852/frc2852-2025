// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Colors;

public class LED extends SubsystemBase {
  private final PWM blinkin;

  public LED() {
      blinkin = new PWM(0);
  }
  //blinkin.setSpeed(pattern.getValue())
  public void setColor(int color){
    blinkin.setSpeed(color);
  }
/*HOT_PINK
DARK_GREEN
VIOLET
TWINKLES_PARTY
*/
  public void setPink(){
    setColor(Colors.HOT_PINK);
  }
  public void setGreen(){
    setColor(Colors.DARK_GREEN);
  }
  public void setViolet(){
    setColor(Colors.VIOLET);
  }
  public void setParty(){
    setColor(Colors.TWINKLES_PARTY);
  }

  
}
