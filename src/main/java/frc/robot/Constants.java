// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }

  public static class CanbusId {
    public static final int INTAKE_MOTOR = 10;
    public static final int WRIST_MOTOR = 11;
    public static final int ELEVATOR_MOTOR = 12;
  }

  public static class MotorSetPoint {
    // Intake Values
    public static final int INTAKE_POSITION_CONVERTION_FACTOR = 0;
    public static final int INTAKE_VELOCITY_CONVERTION_FACTOR = 0;
    public static final int INTAKE_MAX_VELOCITY = 0;
    public static final int INTAKE_MAX_ACCELERATION = 0;
    public static final int INTAKE_ALLOWED_CLOSED_LOOP_ERROR = 2;
    public static final int REVERSE_INTAKE_VELOCITY = -500;
    public static final int INTAKE_VELOCITY = 500;
    public static final int STOP_INTAKE = 0;

    // Wrist Values
    public static final int WRIST_POSITION_CONVERTION_FACTOR = 0;
    public static final int WRIST_VELOCITY_CONVERTION_FACTOR = 0;
    public static final int WRIST_MAX_VELOCITY = 0;
    public static final int WRIST_MAX_ACCELERATION = 0;
    public static final int WRIST_ALLOWED_CLOSED_LOOP_ERROR = 2;
    public static final int BOTTOM_POSITION = 1;
    public static final int WRIST_REEF_LEVEL_1 = 2;
    public static final int WRIST_REEF_LEVEL_2 = 3;
    public static final int WRIST_REEF_LEVEL_3 = 4;
    public static final int WRIST_REEF_LEVEL_4 = 5;
    public static final int WRIST_ALGEA_LEVEL_1 = 6;
    public static final int WRIST_ALGEA_LEVEL_2 = 7;
    public static final int WRIST_ALGEA_LEVEL_3 = 8;
    public static final int WRIST_ALGEA_LEVEL_4 = 9;
    public static final int WRIST_HP_STATION = 10;
    public static final int WRIST_BARGE = 11;
    public static final int WRIST_PROCESSOR = 11;

    // Elevator values
    public static final int ELEVATOR_POSITION = 0;
    public static final int ELEVATOR_POSITION_CONVERSION_FACTOR = 0;
    public static final int ELEVATOR_VELOCITY_CONVERSION_FACTOR = 0;
    public static final int ELEVATOR_CLOSED_LOOP = 0;
    public static final int ELEVATOR_MAX_ACCELERATION = 0;
    public static final int ELEVATOR_MAX_VELOCITY = 0;
    public static final int ELEVATOR_REEF_LEVEL_1 = 0;
    public static final int ELEVATOR_REEF_LEVEL_2 = 0;
    public static final int ELEVATOR_REEF_LEVEL_3 = 0;
    public static final int ELEVATOR_REEF_LEVEL_4 = 0;
    public static final int ELEVATOR_ALGEA_LEVEL_1 = 0;
    public static final int ELEVATOR_ALGEA_LEVEL_2 = 0;
    public static final int ELEVATOR_ALGEA_LEVEL_3 = 0;
    public static final int ELEVATOR_ALGEA_LEVEL_4 = 0;
    public static final int ELEVATOR_HP = 0;
    public static final int ELEVATOR_BARGE = 0;
  }

  public static class Colors {
    public static int HOT_PINK = (int) 0.57; // coral in
    public static int DARK_GREEN = (int) 0.75; // algea in
    public static int VIOLET = (int) 0.91; // coral rollers running
    public static int TWINKLES_PARTY = (int) -0.53;// algea rollers runng
  }

  public enum Side {
    // @formatter:off
      LEFT, 
      RIGHT;
    }

    public enum ScoringLevel {
      // @formatter:off
      LEVEL_1,
      LEVEL_2, 
      LEVEL_3, 
      LEVEL_4
    }

    //Swerve Values
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

}
