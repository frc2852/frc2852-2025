// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;

    // Reef auto drive distance
    public static final double AUTO_DRIVE_DISTANCE_METRES = 3.0;
  }

  public static class PWM {
    public static final int BLINKIN = 0;
  }

  public static class CanbusId {
    public static final int INTAKE_MOTOR = 10;
    public static final int ARM_MOTOR = 11;
    public static final int ELEVATOR_MOTOR = 12;
    public static final int WRIST_MOTOR = 14;
    public static final int ELEVATOR_MOTOR_2 = 15;
    public static final int CLIMBER_MOTOR = 17;

  }

  public static class MotorSetPoint {
    // Intake Values
    public static final int INTAKE_POSITION_CONVERTION_FACTOR = 1;
    public static final int INTAKE_VELOCITY_CONVERTION_FACTOR = 1;
    public static final int INTAKE_MAX_VELOCITY = 6500;
    public static final int INTAKE_MAX_ACCELERATION = 6500;
    public static final int INTAKE_ALLOWED_CLOSED_LOOP_ERROR = 1;

    public static final int INTAKE_VELOCITY_ALGAE = 6000;
    public static final int INTAKE_VELOCITY_CORAL = 5300;
    public static final int INTAKE_VELOCITY_HOLD = 0000;
    public static final int INTAKE_VELOCITY_ALGAE_HOLD = 1700;
    public static final int INTAKE_VELOCITY_REVERSE_CORAL = -3000;
    public static final int INTAKE_VELOCITY_REVERSE_ALGAE = -6500;

    public static final int STOP_INTAKE = 0;

    // Wrist Values
    public static final int WRIST_POSITION_CONVERTION_FACTOR = 1;
    public static final int WRIST_VELOCITY_CONVERTION_FACTOR = 1;
    public static final int WRIST_MAX_VELOCITY = 6000;
    public static final int WRIST_MAX_ACCELERATION = 6000;
    public static final int WRIST_ALLOWED_CLOSED_LOOP_ERROR = 1;

    public static final int WRIST_DRIVE_POSITION = 0;
    public static final int WRIST_SCORE_POSITION = -5;

    // Arm values
    public static final int ARM_POSITION_CONVERTION_FACTOR = 1;
    public static final int ARM_VELOCITY_CONVERTION_FACTOR = 1;
    public static final int ARM_MAX_VELOCITY = 6000;
    public static final int ARM_MAX_ACCELERATION = 6000;
    public static final int ARM_ALLOWED_CLOSED_LOOP_ERROR = 1;

    public static final int ARM_DRIVE_POSITION = 90;
    public static final int ARM_CLIMB_POSITION = 10;

    public static final int ARM_REEF_WAIT = 70;
    public static final int ARM_REEF_LEVEL_1 = 60;
    public static final int ARM_REEF_LEVEL_1_MANUAL = 50;

    public static final int ARM_REEF_LEVEL_2 = 50;
    public static final int ARM_REEF_LEVEL_2_MANUAL = 20;

    public static final int ARM_REEF_LEVEL_3 = 60;
    public static final int ARM_REEF_LEVEL_3_MANUAL = 20;

    public static final int ARM_REEF_LEVEL_4 = 50;
    public static final int ARM_REEF_LEVEL_4_MANUAL = 20;
    public static final int ARM_REEF_WAIT_LEVEL4 = 55;

    public static final int ARM_ALGEA_LEVEL_1 = 25;
    public static final int ARM_ALGEA_LEVEL_2 = 25;
    public static final int ARM_INTAKE_STATION = 140;

    public static final int ARM_WASTE = 130;

    public static final int ARM_FLOOR_PICKUP = 150;
    public static final int ARM_BARGE = 67;
    public static final int ARM_BARGE_CATAPULT = 75;
    public static final int ARM_PROCESSOR = 27;

    // Climber values
    public static final int CLIMBER_POSITION_CONVERTION_FACTOR = 90;
    public static final int CLIMBER_VELOCITY_CONVERTION_FACTOR = 90;
    public static final int CLIMBER_MAX_VELOCITY = 6000;
    public static final int CLIMBER_MAX_ACCELERATION = 6000;
    public static final int CLIMBER_ALLOWED_CLOSED_LOOP_ERROR = 1;

    public static final double CLIMBER_GRAB_POSITION = 17;
    public static final double CLIMBER_DRIVE_POSITION = 9.5;
    public static final double CLIMBER_CLIMB_POSITION = 1;

    // Elevator 
    public static final int ELEVATOR_POSITION_CONVERSION_FACTOR = 1;
    public static final int ELEVATOR_VELOCITY_CONVERSION_FACTOR = 1;
    public static final int ELEVATOR_MAX_ACCELERATION = 3000;
    public static final int ELEVATOR_MAX_VELOCITY = 3500;
    public static final int ELEVATOR_CLOSED_LOOP = 1;

    public static final int ELEVATOR_DRIVE_POSITION = 0;
    public static final int ELEVATOR_CLIMB_POSITION = 2;

    public static final double ELEVATOR_REEF_LEVEL_1 = 0;
    public static final double ELEVATOR_REEF_LEVEL_2 = 10;
    public static final double ELEVATOR_REEF_LEVEL_2_PASS = 15;
    public static final double ELEVATOR_REEF_LEVEL_3 = 20;
    public static final double ELEVATOR_REEF_LEVEL_4 = 36;
    public static final double ELEVATOR_ALGEA_LEVEL_1 = 14;
    public static final double ELEVATOR_ALGEA_LEVEL_2 = 21.5;
    public static final double ELEVATOR_INTAKE_STATION = 13.25;
    public static final double ELEVATOR_BARGE = 37.5;
    public static final double ELEVATOR_MAX_HEIGHT = 38.5;
  }

  public static class RobotSpecifications {

    // TODO: Update this weight, should include weight of bumpers, battery,
    // everything
    public static final double ROBOT_TOTAL_LBS = 150;

    // 32lbs * kg per pound
    public static final double ROBOT_MASS = Units.lbsToKilograms(ROBOT_TOTAL_LBS);

    // Frame perimieter 26"
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(26)), ROBOT_MASS);

    // s, 20ms + 110ms sprk max velocity lag
    public static final double LOOP_TIME = 0.13;

    // NEO Vortex MK4i Drivetrain Free Speed (ft/s)
    // L1 : 14.5
    // L2 : 17.6
    // L3 : 19.3
    // L4 : 23.0
    public static final double MAX_SPEED = Units.feetToMeters(17.6);
    public static final double WHEEL_LOCK_TIME = 10;
  }

  public static class PoseMappings {

    // Translations for position calculation mapping
    public static final Map<String, Translation2d> reefPoses = new HashMap<>();
    public static final Map<String, Translation2d> hsTriangleRed = new HashMap<>();
    public static final Map<String, Translation2d> hsTriangleBlue = new HashMap<>();

    // Pose2d for robot positions
    public static final Map<String, Pose2d> coralPosesRed = new HashMap<>();
    public static final Map<String, Pose2d> coralPosesBlue = new HashMap<>();
    public static final Map<String, Pose2d> algaePosesRed = new HashMap<>();
    public static final Map<String, Pose2d> algaePosesBlue = new HashMap<>();
    public static final Map<String, Pose2d> hsPosesRed = new HashMap<>();
    public static final Map<String, Pose2d> hsPosesBlue = new HashMap<>();

    static {
      // Initialize reef poses
      reefPoses.put("RedReef", new Translation2d(13.063, 4));
      reefPoses.put("BlueReef", new Translation2d(4.5, 4));

      // Initialize coral poses for Red Alliance
      coralPosesRed.put("A", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("B", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("C", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("D", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("E", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("F", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("G", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("H", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("I", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("J", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("K", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      coralPosesRed.put("L", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));

      // Add Habitat Station poses for Red Alliance
      hsPosesRed.put("HS_1_RED", new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
      hsPosesRed.put("HS_2_RED", new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));

      // Add Habitat Station poses for Blue Alliance
      hsPosesBlue.put("HS_1_BLUE", new Pose2d(new Translation2d(1.591, 7.378), Rotation2d.fromDegrees(38)));
      hsPosesBlue.put("HS_2_BLUE", new Pose2d(new Translation2d(1.361, 0.818), Rotation2d.fromDegrees(144)));

      // Initialize coral poses for Blue Alliance
      coralPosesBlue.put("A", new Pose2d(new Translation2d(3.216, 4.194), Rotation2d.fromDegrees(90)));
      coralPosesBlue.put("B", new Pose2d(new Translation2d(3.215, 3.864), Rotation2d.fromDegrees(90)));
      coralPosesBlue.put("C", new Pose2d(new Translation2d(3.705, 3.005), Rotation2d.fromDegrees(150)));
      coralPosesBlue.put("D", new Pose2d(new Translation2d(3.995, 2.848), Rotation2d.fromDegrees(150)));
      coralPosesBlue.put("E", new Pose2d(new Translation2d(4.975, 2.856), Rotation2d.fromDegrees(-150)));
      coralPosesBlue.put("F", new Pose2d(new Translation2d(5.238, 3.007), Rotation2d.fromDegrees(-150)));
      coralPosesBlue.put("G", new Pose2d(new Translation2d(5.761, 3.854), Rotation2d.fromDegrees(-90)));
      coralPosesBlue.put("H", new Pose2d(new Translation2d(5.761, 4.187), Rotation2d.fromDegrees(-90)));
      coralPosesBlue.put("I", new Pose2d(new Translation2d(5.273, 5.033), Rotation2d.fromDegrees(-30)));
      coralPosesBlue.put("J", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-30)));
      coralPosesBlue.put("K", new Pose2d(new Translation2d(4.004, 5.2), Rotation2d.fromDegrees(30)));
      coralPosesBlue.put("L", new Pose2d(new Translation2d(4.004, 5.206), Rotation2d.fromDegrees(30)));

      // Initialize algae poses for Red Alliance
      algaePosesRed.put("A-B", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesRed.put("C-D", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesRed.put("E-F", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesRed.put("G-H", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesRed.put("I-J", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesRed.put("K-L", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));

      // Initialize algae poses for Blue Alliance
      algaePosesBlue.put("A-B", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesBlue.put("C-D", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesBlue.put("E-F", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesBlue.put("G-H", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesBlue.put("I-J", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
      algaePosesBlue.put("K-L", new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));

      // Initialize triangle poses for Red Alliance
      hsTriangleRed.put("Triangle1A", new Translation2d(17.5, 4.0));
      hsTriangleRed.put("Triangle1B", new Translation2d(13.5, 8.0));
      hsTriangleRed.put("Triangle1C", new Translation2d(17.5, 8.0));

      hsTriangleRed.put("Triangle2A", new Translation2d(17.5, 4.0));
      hsTriangleRed.put("Triangle2B", new Translation2d(13.5, 0.0));
      hsTriangleRed.put("Triangle2C", new Translation2d(17.5, 0.0));

      // Initialize triangle poses for Blue Alliance
      hsTriangleBlue.put("Triangle1A", new Translation2d(0.0, 4.0));
      hsTriangleBlue.put("Triangle1B", new Translation2d(4.0, 8.0));
      hsTriangleBlue.put("Triangle1C", new Translation2d(0.0, 8.0));

      hsTriangleBlue.put("Triangle2A", new Translation2d(0.0, 4.0));
      hsTriangleBlue.put("Triangle2B", new Translation2d(4.0, 0.0));
      hsTriangleBlue.put("Triangle2C", new Translation2d(0.0, 0.0));
    }
  }

  public enum ScoringLevel {
    // @formatter:off
    LEVEL_1,
    LEVEL_2, 
    LEVEL_3, 
    LEVEL_4
  }

  public enum ScoringZone {
    // @formatter:off
    UNKNOWN,
    ZONE_AB,
    ZONE_CD, 
    ZONE_EF, 
    ZONE_GH,
    ZONE_IJ,
    ZONE_KL,
    HS_BLUE_1,
    HS_BLUE_2,
    HS_RED_1,
    HS_RED_2
  }

  public enum Side {
    // @formatter:off
    LEFT, 
    RIGHT
  }
  
  public enum Pattern {
    // @formatter:off
    RAINBOW(-0.99),
    PARTY_RAINBOW(-0.97),
    OCEAN_RAINBOW(-0.95),
    LAVA_RAINBOW(-0.93),
    FOREST_RAINBOW(-0.91),
    GLITTER_RAINBOW(-0.89),
    CONFETTI(-0.87),
    SHOT_RED(-0.85),
    SHOT_BLUE(-0.83),
    SHOT_WHITE(-0.81),
    SINELON_RAINBOW(-0.79),
    SINELON_PARTY(-0.77),
    SINELON_OCEAN(-0.75),
    SINELON_LAVA(-0.73),
    SINELON_FOREST(-0.71),
    BPM_RAINBOW(-0.69),
    BPM_PARTY(-0.67),
    BPM_OCEAN(-0.65),
    BPM_LAVA(-0.63),
    BPM_FOREST(-0.61),
    FIRE_MEDIUM(-0.59),
    FIRE_LARGE(-0.57),
    TWINKLES_RAINBOW(-0.55),
    TWINKLES_PARTY(-0.53),
    TWINKLES_OCEAN(-0.51),
    TWINKLES_LAVA(-0.49),
    TWINKLES_FOREST(-0.47),
    COLOR_WAVES_RAINBOW(-0.45),
    COLOR_WAVES_PARTY(-0.43),
    COLOR_WAVES_OCEAN(-0.41),
    COLOR_WAVES_LAVA(-0.39),
    COLOR_WAVES_FOREST(-0.37),
    LARSON_SCANNER_RED(-0.35),
    LARSON_SCANNER_GRAY(-0.33),
    LIGHT_CHASE_RED(-0.31),
    LIGHT_CHASE_BLUE(-0.29),
    LIGHT_CHASE_GRAY(-0.27),
    HEARTBEAT_RED(-0.25),
    HEARTBEAT_BLUE(-0.23),
    HEARTBEAT_WHITE(-0.21),
    HEARTBEAT_GRAY(-0.19),
    BREATH_RED(-0.17),
    BREATH_BLUE(-0.15),
    BREATH_GRAY(-0.13),
    STROBE_RED(-0.11),
    STROBE_BLUE(-0.09),
    STROBE_GOLD(-0.07),
    STROBE_WHITE(-0.05),
    HOT_PINK(0.57),
    DARK_RED(0.59),
    RED(0.61),
    RED_ORANGE(0.63),
    ORANGE(0.65),
    GOLD(0.67),
    YELLOW(0.69),
    LAWN_GREEN(0.71),
    LIME(0.73),
    DARK_GREEN(0.75),
    GREEN(0.77),
    BLUE_GREEN(0.79),
    AQUA(0.81),
    SKY_BLUE(0.83),
    DARK_BLUE(0.85),
    BLUE(0.87),
    BLUE_VIOLET(0.89),
    VIOLET(0.91),
    WHITE(0.93),
    GRAY(0.95),
    DARK_GRAY(0.97),
    OFF(0.99);

    private final double value;

    Pattern(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
  }
}
