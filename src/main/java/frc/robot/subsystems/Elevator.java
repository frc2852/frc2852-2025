package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

  private final RelativeEncoder encoder;

  private final double P = 0.1;
  private final double I = 0;
  private final double D = 0;
  private final double OUTPUT_RANGE = 1;

  private double targetPosition;
  private double manualPosition = 0;

  public Elevator() {
    // Configure motor
    motor = new SparkFlex(CanbusId.ELEVATOR_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Configure motor properties
    motorConfig = new SparkFlexConfig();

    // Configure motor properties
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    // Configure encoder conversion factors
    motorConfig.encoder
        .positionConversionFactor(MotorSetPoint.ELEVATOR_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.ELEVATOR_VELOCITY_CONVERSION_FACTOR);

    // Configure closed loop PID using the relative encoder (now in degrees).
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P)
        .i(I)
        .d(D)
        .outputRange(-OUTPUT_RANGE, OUTPUT_RANGE);

    // cONFIGURE Max motion
    motorConfig.closedLoop.maxMotion
        .maxVelocity(MotorSetPoint.ELEVATOR_MAX_VELOCITY)
        .maxAcceleration(MotorSetPoint.ELEVATOR_MAX_ACCELERATION)
        .allowedClosedLoopError(MotorSetPoint.ELEVATOR_CLOSED_LOOP);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("ElevatorManualPosition", manualPosition);
    }
  }

  /**
   * Command the Elevator to go to a specific position.
   *
   * @param position The target Elevator angle in degrees.
   */
  public void goToPosition(double position) {
    targetPosition = position;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - targetPosition) <= 2;
  }

  @Override
  public void periodic() {
    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("ElevatorPosition", encoder.getPosition());
      SmartDashboard.putNumber("ElevatorCurrent", motor.getOutputCurrent());
      SmartDashboard.putNumber("ElevatorTemperature", motor.getMotorTemperature());
      SmartDashboard.putBoolean("ElevatorAtPosition", isAtPosition());

      manualPosition = SmartDashboard.getNumber("ElevatorManualPosition", manualPosition);
      goToPosition(manualPosition);
    }
  }
}
