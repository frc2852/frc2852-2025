package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PoseMappings;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.ScoringZone;
import frc.robot.Constants.Side;
import frc.robot.data.RobotControlStateSendable;

public class RobotControlState {

  private static Optional<Alliance> alliance;
  private static ScoringZone zone = ScoringZone.UNKNOWN;
  private static ScoringLevel scoringLevel = ScoringLevel.LEVEL_1;
  private static Side side = Side.LEFT;

  private static boolean climbEnabled = false;
  private static Translation2d allianceReef = null;

  // Angle offset to align with the opposite side of the scoring table.
  public static double angleOffset = 90;

  private static final RobotControlStateSendable robotControlStateSendable = new RobotControlStateSendable();

  private RobotControlState() {
  }

  public static void periodic() {
    SmartDashboard.putData("Robot Control State", robotControlStateSendable);

    // Add boolean indicators for each scoring level.
    SmartDashboard.putBoolean("Level1", scoringLevel == ScoringLevel.LEVEL_1);
    SmartDashboard.putBoolean("Level2", scoringLevel == ScoringLevel.LEVEL_2);
    SmartDashboard.putBoolean("Level3", scoringLevel == ScoringLevel.LEVEL_3);
    SmartDashboard.putBoolean("Level4", scoringLevel == ScoringLevel.LEVEL_4);

    SmartDashboard.putString("Zone", getAllianceZone());
    SmartDashboard.putBoolean("ClimbEnabled", climbEnabled);
    SmartDashboard.putString("Side", side.name());
  }

  public static ScoringLevel getScoringLevel() {
    return scoringLevel;
  }

  public static void setScoringLevel(ScoringLevel newScoringLevel) {
    scoringLevel = newScoringLevel;
  }

  public static Side getSide() {
    return side;
  }

  public static void setSide(Side newSide) {
    side = newSide;
  }

  /**
   * Returns the alliance-specific zone identifier based on the current alliance
   * and zone.
   *
   * @return The alliance zone identifier or "UNKNOWN" if not found.
   */
  public static String getAllianceZone() {
    if (!alliance.isPresent()) {
      return "UNKNOWN";
    }

    // Define the zone mappings for each alliance.
    Map<ScoringZone, String> redAllianceZones = Map.of(
        ScoringZone.ZONE_AB, "ZONE_2",
        ScoringZone.ZONE_CD, "ZONE_1",
        ScoringZone.ZONE_EF, "ZONE_6",
        ScoringZone.ZONE_GH, "ZONE_5",
        ScoringZone.ZONE_IJ, "ZONE_4",
        ScoringZone.ZONE_KL, "ZONE_3",
        ScoringZone.HS_RED_1, "HS_RED_1",
        ScoringZone.HS_RED_2, "HS_RED_2");

    Map<ScoringZone, String> blueAllianceZones = Map.of(
        ScoringZone.ZONE_AB, "ZONE_5",
        ScoringZone.ZONE_CD, "ZONE_4",
        ScoringZone.ZONE_EF, "ZONE_3",
        ScoringZone.ZONE_GH, "ZONE_2",
        ScoringZone.ZONE_IJ, "ZONE_1",
        ScoringZone.ZONE_KL, "ZONE_6",
        ScoringZone.HS_BLUE_1, "HS_BLUE_1",
        ScoringZone.HS_BLUE_2, "HS_BLUE_2");

    // Select the appropriate map based on alliance.
    Map<ScoringZone, String> zoneMap = (alliance.get() == Alliance.Red) ? redAllianceZones : blueAllianceZones;

    // Return the corresponding zone or "UNKNOWN" if the zone is not mapped.
    return zoneMap.getOrDefault(zone, "UNKNOWN");
  }

  public static ScoringZone getZone() {
    return zone;
  }

  public static void setZone(ScoringZone newZone) {
    zone = newZone;
  }

  public static boolean isClimbEnabled() {
    return climbEnabled;
  }

  public static void toggleClimbEnabled() {
    climbEnabled = !climbEnabled;
  }

  public static String getAlliance() {
    if (alliance == null || alliance.isEmpty()) {
      return "UNKNOWN";
    }
    return alliance.get().name();
  }

  public static void setAlliance() {
    RobotControlState.alliance = DriverStation.getAlliance();
  }

  /**
   * Returns the appropriate Pose2d for the current scoring zone (coral or HS)
   * based on the current alliance, zone, and side.
   *
   * @return the selected Pose2d or null if not found.
   */
  public static Pose2d getZonePose() {
    // Ensure alliance information is set
    if (alliance == null || alliance.isEmpty()) {
      setAlliance();
    }

    // Handle coral zones
    if (zone == ScoringZone.ZONE_AB ||
        zone == ScoringZone.ZONE_CD ||
        zone == ScoringZone.ZONE_EF ||
        zone == ScoringZone.ZONE_GH ||
        zone == ScoringZone.ZONE_IJ ||
        zone == ScoringZone.ZONE_KL) {

      // Reuse the existing coral pose logic
      Map<String, Pose2d> coralMap = getCoralMapForAlliance();
      if (coralMap == null) {
        return null;
      }

      String letter = getCoralLetter(zone, side);
      if (letter == null) {
        return null;
      }

      return coralMap.get(letter);
    }
    // Handle Habitat Station (HS) zones
    else if (zone == ScoringZone.HS_RED_1) {
      return PoseMappings.hsPosesRed.get("HS_1_RED");
    } else if (zone == ScoringZone.HS_RED_2) {
      return PoseMappings.hsPosesRed.get("HS_2_RED");
    } else if (zone == ScoringZone.HS_BLUE_1) {
      return PoseMappings.hsPosesBlue.get("HS_1_BLUE");
    } else if (zone == ScoringZone.HS_BLUE_2) {
      return PoseMappings.hsPosesBlue.get("HS_2_BLUE");
    }

    return null;
  }

  /**
   * Returns the coral map corresponding to the current alliance.
   *
   * @return the coral map or null if the alliance is not set or invalid.
   */
  private static Map<String, Pose2d> getCoralMapForAlliance() {

    if (alliance == null || alliance.isEmpty()) {
      setAlliance();
    }

    return (alliance.get() == Alliance.Red) ? PoseMappings.coralPosesRed : PoseMappings.coralPosesBlue;
  }

  /**
   * Determines the coral letter based on the scoring zone and side.
   *
   * @param zone
   *             the current scoring zone.
   * @param side
   *             the side (LEFT or RIGHT).
   * @return the corresponding letter or null if the zone is unknown.
   */
  private static String getCoralLetter(ScoringZone zone, Side side) {
    Map<ScoringZone, String[]> zoneToLetters = Map.of(
        ScoringZone.ZONE_AB, new String[] { "A", "B" },
        ScoringZone.ZONE_CD, new String[] { "C", "D" },
        ScoringZone.ZONE_EF, new String[] { "E", "F" },
        ScoringZone.ZONE_GH, new String[] { "G", "H" },
        ScoringZone.ZONE_IJ, new String[] { "I", "J" },
        ScoringZone.ZONE_KL, new String[] { "K", "L" });

    String[] letters = zoneToLetters.get(zone);
    if (letters == null) {
      return null;
    }

    return (side == Side.LEFT) ? letters[0] : letters[1];
  }

  /**
   * Updates the current scoring zone based on the robot's Pose2d.
   * Determines the scoring zone based on the robot's position relative to the
   * reef,
   * including both circular distance from the reef and checks within triangle
   * zones.
   * Process:
   * - Ensures alliance and reef information are set.
   * - Computes the distance from the robot to the reef.
   * - If the robot is within a certain distance (AUTO_DRIVE_DISTANCE) from the
   * reef:
   * calculates the angle from the reef to the robot to determine which circular
   * zone
   * the robot is in. Each circular zone is a 60-degree slice around the reef.
   * - Regardless of distance, also checks if the robot is inside any of the
   * special
   * triangular zones near the reef using point-in-triangle math.
   * The math:
   * - Uses the distance formula (from the difference vector's norm) to check
   * proximity.
   * - Uses angle calculations and sectoring (dividing 360° by 60° segments) to
   * find zones.
   * - For triangles, relies on area comparisons to determine if the robot's
   * position
   * falls inside one of the triangles.
   *
   * @param robotPose
   *                  The current Pose2d of the robot.
   */
  public static void updateZone(Pose2d robotPose) {
    if (alliance == null || alliance.isEmpty()) {
      setAlliance();
    }

    if (allianceReef == null) {
      allianceReef = getAllianceReef();
    }

    Translation2d robotPos = robotPose.getTranslation();
    Translation2d diff = robotPos.minus(allianceReef);
    double distance = diff.getNorm();

    // Initialize zone as UNKNOWN.
    ScoringZone newZone = ScoringZone.UNKNOWN;

    // Determine the circular zone only if within the specified distance.
    if (distance <= OperatorConstants.AUTO_DRIVE_DISTANCE) {
      double angleDeg = calculateNormalizedAngle(diff);
      newZone = determineZone(angleDeg, alliance.get());
      SmartDashboard.putNumber("Angle", angleDeg);
    }

    // Always check triangle zones for the current alliance.
    ScoringZone triangleZone = checkTriangleZones(robotPos);
    if (triangleZone != null) {
      newZone = triangleZone;
    }

    zone = newZone;
  }

  /**
   * Determines the reef pose based on the alliance.
   *
   * @return The reef pose for the current alliance.
   */
  private static Translation2d getAllianceReef() {
    return (alliance.get() == Alliance.Red)
        ? PoseMappings.reefPoses.get("RedReef")
        : PoseMappings.reefPoses.get("BlueReef");
  }

  /**
   * Calculates the robot's angle relative to the reef, normalized to a 0-360
   * degree range,
   * and then adjusts it with an offset to align with scoring zones.
   * Process:
   * - First, computes the angle in radians between the horizontal axis and the
   * vector from reef to robot.
   * - Converts that angle into degrees.
   * - Normalizes the angle to a 0-360 degree range (if negative, adds 360).
   * - Adjusts the angle by subtracting a constant offset and converting
   * from the standard counterclockwise measurement to a clockwise measurement.
   * - Ensures the final angle remains within 0-360 degrees.
   * Why do this?
   * - This normalized angle is used to determine which of the six equal 60-degree
   * zones
   * the robot is in around the reef.
   *
   * @param diff
   *             The vector from the reef to the robot.
   * @return The normalized and adjusted angle in degrees.
   */
  private static double calculateNormalizedAngle(Translation2d diff) {
    double angleRad = Math.atan2(diff.getY(), diff.getX());
    double angleDeg = Math.toDegrees(angleRad);
    angleDeg = (angleDeg < 0) ? angleDeg + 360 : angleDeg;

    // Apply angle offset and convert to clockwise
    angleDeg = (360 - ((angleDeg - angleOffset + 360) % 360)) % 360;

    return angleDeg;
  }

  /**
   * Determines which scoring zone the robot is in based on its normalized angle
   * from the reef and alliance.
   * Process:
   * - Depending on the alliance, uses an array of zones that correspond to
   * 60-degree slices of a circle.
   * - Divides the full 360 degrees by 60 to get one of six segments (0 through
   * 5).
   * - Selects the scoring zone corresponding to that segment from the array.
   * The math:
   * - Uses floor division of the angle by 60 to map continuous angle to discrete
   * zone segments.
   * For example, if angleDeg is 45°, then floor(45/60)=0, which corresponds to
   * the first zone in the list.
   *
   * @param angleDeg
   *                 The normalized angle in degrees relative to the reef.
   * @param alliance
   *                 The current alliance.
   * @return The corresponding scoring zone based on the angle.
   */
  private static ScoringZone determineZone(double angleDeg, Alliance alliance) {
    ScoringZone[] zones = (alliance == Alliance.Red)
        ? new ScoringZone[] { ScoringZone.ZONE_CD, ScoringZone.ZONE_AB, ScoringZone.ZONE_KL, ScoringZone.ZONE_IJ,
            ScoringZone.ZONE_GH, ScoringZone.ZONE_EF }
        : new ScoringZone[] { ScoringZone.ZONE_IJ, ScoringZone.ZONE_GH, ScoringZone.ZONE_EF, ScoringZone.ZONE_CD,
            ScoringZone.ZONE_AB, ScoringZone.ZONE_KL };

    int zoneIndex = (int) Math.floor(angleDeg / 60);
    return (zoneIndex >= 0 && zoneIndex < zones.length) ? zones[zoneIndex] : ScoringZone.UNKNOWN;
  }

  /**
   * Checks if the robot's position falls within any of the predefined triangular
   * scoring zones.
   * Process:
   * - Depending on the alliance (Red or Blue), selects the corresponding set of
   * triangle vertices.
   * - For each triangle zone, retrieves its three corner points.
   * - Uses the pointInTriangle method to check if the robot lies inside the
   * triangle.
   * The math:
   * - It calculates the area of the triangle formed by the triangle’s vertices.
   * - Then calculates areas of three smaller triangles formed by replacing one
   * vertex
   * with the robot's position.
   * - If the sum of these smaller areas equals the area of the whole triangle
   * (within a tiny margin of error), the robot is inside that triangle.
   *
   * @param robotPos
   *                 The current position of the robot.
   * @return The corresponding scoring zone if inside a triangle; otherwise null.
   */
  private static ScoringZone checkTriangleZones(Translation2d robotPos) {
    Alliance currentAlliance = alliance.get();
    Map<String, Translation2d> trianglePoses;
    ScoringZone firstZone;
    ScoringZone secondZone;

    if (currentAlliance == Alliance.Red) {
      trianglePoses = PoseMappings.hsTriangleRed;
      firstZone = ScoringZone.HS_RED_1;
      secondZone = ScoringZone.HS_RED_2;
    } else {
      trianglePoses = PoseMappings.hsTriangleBlue;
      firstZone = ScoringZone.HS_BLUE_1;
      secondZone = ScoringZone.HS_BLUE_2;
    }

    Translation2d first1 = trianglePoses.get("Triangle1A");
    Translation2d first2 = trianglePoses.get("Triangle1B");
    Translation2d first3 = trianglePoses.get("Triangle1C");
    if (pointInTriangle(robotPos, first1, first2, first3)) {
      return firstZone;
    }

    Translation2d second1 = trianglePoses.get("Triangle2A");
    Translation2d second2 = trianglePoses.get("Triangle2B");
    Translation2d second3 = trianglePoses.get("Triangle2C");
    if (pointInTriangle(robotPos, second1, second2, second3)) {
      return secondZone;
    }

    return null;
  }

  /**
   * Checks if a given point p lies inside the triangle defined by points a, b,
   * and c.
   * Math explanation:
   * - It computes the area of the main triangle abc.
   * - Then computes the areas of three smaller triangles: pbc, apc, and abp.
   * - If the sum of these smaller areas equals the area of the main triangle
   * (within a small tolerance to account for floating point precision), then
   * point p
   * lies inside the triangle.
   * Why does this work?
   * - If p is inside triangle abc, then the smaller triangles cover the exact
   * area of abc.
   * If p is outside, the sum would be larger.
   *
   * @param p
   *          The point to check.
   * @param a
   *          The first vertex of the triangle.
   * @param b
   *          The second vertex of the triangle.
   * @param c
   *          The third vertex of the triangle.
   * @return True if point p is inside the triangle, false otherwise.
   */
  private static boolean pointInTriangle(Translation2d p, Translation2d a, Translation2d b, Translation2d c) {
    double area = triangleArea(a, b, c);
    double area1 = triangleArea(p, b, c);
    double area2 = triangleArea(a, p, c);
    double area3 = triangleArea(a, b, p);
    // The point is inside the triangle if the areas add up (within a small
    // tolerance).
    return Math.abs(area - (area1 + area2 + area3)) < 1e-6;
  }

  /**
   * Computes the absolute area of the triangle defined by points p1, p2, and p3
   * using the
   * shoelace formula.
   * Math explanation:
   * - Given three points (x1, y1), (x2, y2), and (x3, y3),
   * the area is |x1(y2 - y3) + x2(y3 - y1) + x3(y1 - y2)| / 2.
   * - This formula effectively calculates a determinant that gives twice the
   * area,
   * so dividing by 2 and taking the absolute value yields the triangle's area.
   *
   * @param p1
   *           The first vertex of the triangle.
   * @param p2
   *           The second vertex of the triangle.
   * @param p3
   *           The third vertex of the triangle.
   * @return The area of the triangle.
   */
  private static double triangleArea(Translation2d p1, Translation2d p2, Translation2d p3) {
    return Math.abs(
        p1.getX() * (p2.getY() - p3.getY()) +
            p2.getX() * (p3.getY() - p1.getY()) +
            p3.getX() * (p1.getY() - p2.getY()))
        / 2.0;
  }
}
