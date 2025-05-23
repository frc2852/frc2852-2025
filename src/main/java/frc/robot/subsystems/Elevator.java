package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.ElevatorFeedforward;
import frc.robot.Constants.ElevatorFeedforwardConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // For logging if needed, or just for test commands
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class Elevator extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController controller;

  private final SparkFlex motorFollow;
  private final SparkFlexConfig motorFollowConfig;

  private final RelativeEncoder encoder;
  private final RelativeEncoder encoderFollow;

  private double P = 0.1;
  private double I = 0.0;
  private double D = 0.001;
  private final double outputRange = 1.0;

  private double maxVelocity = MotorSetPoint.ELEVATOR_MAX_VELOCITY;
  private double maxAcceleration = MotorSetPoint.ELEVATOR_MAX_ACCELERATION;

  private double targetPosition;

  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State previousState = new TrapezoidProfile.State();
  private ElevatorFeedforward feedforward;

  public Elevator() {
    // Configure motor
    motor = new SparkFlex(CanbusId.ELEVATOR_MOTOR, MotorType.kBrushless);
    motorFollow = new SparkFlex(CanbusId.ELEVATOR_MOTOR_2, MotorType.kBrushless);

    // Closed loop controller
    controller = motor.getClosedLoopController();

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);
    encoderFollow = motorFollow.getEncoder();
    encoderFollow.setPosition(0);

    // Configure motor properties
    motorConfig = new SparkFlexConfig();
    motorFollowConfig = new SparkFlexConfig();

    // Configure motor properties
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(80);

    motorFollowConfig.idleMode(IdleMode.kBrake);
    motorFollowConfig.follow(motor, true);
    motorFollowConfig.smartCurrentLimit(80);

    // Configure encoder conversion factors
    motorConfig.encoder
        .positionConversionFactor(MotorSetPoint.ELEVATOR_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.ELEVATOR_VELOCITY_CONVERSION_FACTOR);

    // Configure closed-loop PID and output
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P)
        .i(I)
        .d(D)
        .outputRange(-outputRange, outputRange);

    // Apply configuration.
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFollow.configure(motorFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize TrapezoidProfile constraints and feedforward
    constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    feedforward = new ElevatorFeedforward(
        ElevatorFeedforwardConstants.kS,
        ElevatorFeedforwardConstants.kG,
        ElevatorFeedforwardConstants.kV,
        ElevatorFeedforwardConstants.kA
    );
    // Initialize previousState. Make sure encoder is reset beforehand or starting position is known.
    previousState = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
  }

  /**
   * Command the Elevator to go to a specific position.
   *
   * @param position The target Elevator angle in degrees.
   */
  public void goToPosition(double position) {
    if (position > 37.5) {
      position = 37.5;
    } else if (position <= 0) {
      position = 0;
    }
    targetPosition = position;
    goalState = new TrapezoidProfile.State(targetPosition, 0); // Target velocity is 0
    // controller.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.59); // Removed
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - targetPosition) <= 2;
  }

  public boolean isAtDrivePosition() {
    return targetPosition == MotorSetPoint.ELEVATOR_DRIVE_POSITION;
  }

  @Override
  public void periodic() {
      // Create a new profile starting from the previous state, moving towards the goal state
      TrapezoidProfile profile = new TrapezoidProfile(constraints, goalState, previousState);

      // Calculate the state for the next timestep (assuming a 20ms loop time)
      TrapezoidProfile.State nextState = profile.calculate(0.02); // 0.02 seconds = 20ms

      // Calculate feedforward
      // Note: nextState.velocity is speed, acceleration needs to be calculated or taken from profile
      // The TrapezoidProfile class itself doesn't directly expose acceleration for the next segment in a simple way.
      // We can calculate acceleration based on velocity change over the time step.
      double currentAcceleration = (nextState.velocity - previousState.velocity) / 0.02;
      double feedforwardOutput = feedforward.calculate(nextState.velocity, currentAcceleration);

      // Update motor setpoint using kPosition
      // The last parameter (arbFeedforward) is where the calculated feedforward goes.
      controller.setReference(nextState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardOutput);

      // Update previousState for the next iteration
      previousState = nextState;
  }

  /**
   * Sets the raw voltage to the elevator motor. Used for SysId characterization.
   *
   * @param voltage The voltage to apply to the motor (typically -12.0 to 12.0).
   */
  public void setVoltage(double voltage) {
      controller.setReference(voltage, ControlType.kVoltage, ClosedLoopSlot.kSlot0, 0);
  }

  /**
   * Logs data useful for SysId characterization to SmartDashboard.
   * Call this method in periodic() when running SysId tests.
   */
  public void logSysIdData() {
      SmartDashboard.putNumber("Elevator/SysId/Position", encoder.getPosition());
      SmartDashboard.putNumber("Elevator/SysId/Velocity", encoder.getVelocity());
      // Applied voltage might need to be tracked separately if not directly available from motor controller
      // For now, assume the commanded voltage is close enough for initial tests or can be logged by the caller.
      // SmartDashboard.putNumber("Elevator/SysId/AppliedVoltage", motor.getAppliedOutput() * motor.getBusVoltage()); // Example
  }

  /**
   * @return The current position of the elevator encoder.
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * @return The current velocity of the elevator encoder.
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }
}
