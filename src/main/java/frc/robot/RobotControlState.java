// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.Side;

/** Add your docs here. */
public class RobotControlState {

  private static Side side;
    private static ScoringLevel scoringLevel;
    
      public Side getSide() {
        return side;
      }
    
      public ScoringLevel getScoringLevel() {
        return scoringLevel;
      }
    
      public static void setSide(Side newSide) {
        side = newSide;
    }
  
    public static void setScoringLevel(ScoringLevel newScoringLevel) {
      scoringLevel = newScoringLevel;
  }

}
