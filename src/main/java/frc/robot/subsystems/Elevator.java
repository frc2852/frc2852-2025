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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double manualPosition = 0;

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

    // Configure motion profile limits
    motorConfig.closedLoop.maxMotion
        .maxVelocity(maxVelocity)
        .maxAcceleration(maxAcceleration)
        .allowedClosedLoopError(MotorSetPoint.ELEVATOR_CLOSED_LOOP);

    // Apply configuration.
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFollow.configure(motorFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Publish initial values to SmartDashboard if in test mode.
    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("ElevatorP", P);
      SmartDashboard.putNumber("ElevatorI", I);
      SmartDashboard.putNumber("ElevatorD", D);
      SmartDashboard.putNumber("ElevatorMaxVelocity", maxVelocity);
      SmartDashboard.putNumber("ElevatorMaxAcceleration", maxAcceleration);
      SmartDashboard.putNumber("ElevatorManualPosition", manualPosition);
    }
  }

  /**
   * Command the Elevator to go to a specific position.
   *
   * @param position The target Elevator angle in degrees.
   */
  public void goToPosition(double position) {
    if(position > 37.5) {
      position = 37.5;
    } else if (position <= 0) {
      position = 0;
    }
    targetPosition = position;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.59);
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - targetPosition) <= 2;
  }

  @Override
  public void periodic() {
    if (DriverStation.isTest()) {
      double newP = SmartDashboard.getNumber("ElevatorP", P);
      double newI = SmartDashboard.getNumber("ElevatorI", I);
      double newD = SmartDashboard.getNumber("ElevatorD", D);
      double newMaxVel = SmartDashboard.getNumber("ElevatorMaxVelocity", maxVelocity);
      double newMaxAccel = SmartDashboard.getNumber("ElevatorMaxAcceleration", maxAcceleration);
      manualPosition = SmartDashboard.getNumber("ElevatorManualPosition", manualPosition);

      // If any value has changed, update the closed-loop configuration.
      if (newP != P || newI != I || newD != D ||
          newMaxVel != maxVelocity || newMaxAccel != maxAcceleration) {
        P = newP;
        I = newI;
        D = newD;
        maxVelocity = newMaxVel;
        maxAcceleration = newMaxAccel;

        // Update PID and motion parameters.
        motorConfig.closedLoop
            .p(P)
            .i(I)
            .d(D)
            .outputRange(-outputRange, outputRange);

        motorConfig.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration);

        // Reconfigure the motor with the new settings.
        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }

      // Optionally, update additional telemetry.
      SmartDashboard.putNumber("ElevatorMotor", motor.getAppliedOutput());
      SmartDashboard.putNumber("ElevatorFollowMotor", motorFollow.getAppliedOutput());
      SmartDashboard.putNumber("ElevatorPosition", encoder.getPosition());
      SmartDashboard.putNumber("ElevatorFollowPosition", encoderFollow.getPosition());
      SmartDashboard.putNumber("ElevatorCurrent", motor.getOutputCurrent());
      SmartDashboard.putNumber("ElevatorTemperature", motor.getMotorTemperature());
      SmartDashboard.putBoolean("ElevatorAtPosition", isAtPosition());

      goToPosition(manualPosition);
    }
  }
}
