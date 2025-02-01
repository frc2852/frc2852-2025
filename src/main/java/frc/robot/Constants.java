// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CanbusId{
    public static final int INTAKE_MOTOR=10;
    public static final int WRIST_MOTOR=11;
    //...
  }

  public static class MotorSetPoint{
    public static final int INTAKE_VELOCITY = 500;
    public static final int STOP_INTAKE = 0;
    public static final int BOTTOM_POSITION = 1;
    public static final int REEF_LEVEL_1 = 2;
    public static final int REEF_LEVEL_2 = 3;
    public static final int REEF_LEVEL_3 = 4;
    public static final int REEF_LEVEL_4 = 5;
    public static final int ALGEA_LEVEL_1 = 6;
    public static final int ALGEA_LEVEL_2 = 7;
    public static final int ALGEA_LEVEL_3 = 8;
    public static final int HP_STATION= 9;
    public static final int BARGE = 10;
  }
}
