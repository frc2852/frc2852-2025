// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.Side;

/** Add your docs here. */
public class RobotControlState {

  private static double area;

  private static Side side;
  private static ScoringLevel scoringLevel;

  private static boolean hasAlgea = false;
  private static boolean hasCoral = false;

  public static boolean hasAlgea() {
    return hasAlgea;
  }

  public static void setHasAlgea(boolean value) {
    hasAlgea = value;
  }

  public static boolean hasCoral() {
    return hasCoral;
  }

  public static void setHasCoral(boolean value) {
    hasCoral = value;
  }

  public static Side getSide() {
    return side;
  }

  public static ScoringLevel getScoringLevel() {
    return scoringLevel;
  }

  public static void setSide(Side newSide) {
    side = newSide;
  }

  public static void setScoringLevel(ScoringLevel newScoringLevel) {
    scoringLevel = newScoringLevel;
  }

  public static void areaCalculation(Translation2d p1) {
    double x1 = p1.getX();
    double x2 = p1.getX();
    double x3 = p1.getX();
    double y1 = p1.getY();
    double y2 = p1.getY();
    double y3 = p1.getY();

    double area = Math.abs(x1 * (y1 - y2) + x2 * (y3 - y2) + x3 * (y1 - y3)) / 2;
  }
}
