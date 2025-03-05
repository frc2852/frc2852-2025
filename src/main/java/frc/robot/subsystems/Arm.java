package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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

public class Arm extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController controller;

  private final AbsoluteEncoder absEncoder;
  private final RelativeEncoder encoder;

  private final double P = 0.1;
  private final double I = 0;
  private final double D = 0;
  private final double OUTPUT_RANGE = 0.7;

  private final double GEAR_REDUCTION = 267.86;
  private final double RELATIVE_ENCODER_CONVERSION = 360.0 / GEAR_REDUCTION;

  private double targetPosition;
  private double manualPosition = 90;

  private boolean hasInitializedPosition = false;

  public Arm() {
    motor = new SparkFlex(CanbusId.ARM_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoders
    absEncoder = motor.getAbsoluteEncoder();
    encoder = motor.getEncoder();

    // Set conversion factor for the relative encoder so that getPosition() returns
    // degrees.
    // (This conversion factor converts motor rotations to Arm degrees.)
    motorConfig = new SparkFlexConfig();
    motorConfig.encoder.positionConversionFactor(RELATIVE_ENCODER_CONVERSION);

    // Configure motor properties
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);

    // Configure absolute encoder conversion factor.
    // The absolute encoder returns degrees already (2 rotations = 360° with factor
    // 170.2).
    motorConfig.absoluteEncoder
        .positionConversionFactor(170.2)
        .velocityConversionFactor(1)
        .inverted(true);

    // Configure closed loop PID using the relative encoder (now in degrees).
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P)
        .i(I)
        .d(D)
        .outputRange(-OUTPUT_RANGE, OUTPUT_RANGE);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Command the Arm to go to a specific position.
   *
   * @param position The target Arm angle in degrees.
   */
  public void goToPosition(double position) {
    targetPosition = position;
    controller.setReference(targetPosition, ControlType.kPosition);
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - targetPosition) <= 2;
  }

  public boolean isAtScoringPosition() {
    return targetPosition == MotorSetPoint.ARM_REEF_LEVEL_1_MANUAL
        || targetPosition == MotorSetPoint.ARM_REEF_LEVEL_2_MANUAL
        || targetPosition == MotorSetPoint.ARM_REEF_LEVEL_3_MANUAL
        || targetPosition == MotorSetPoint.ARM_REEF_LEVEL_4_MANUAL;
  }

  @Override
  public void periodic() {
    if (!hasInitializedPosition && absEncoder.getPosition() > 5) {
      // Initialize the relative encoder using the absolute encoder value.
      encoder.setPosition(absEncoder.getPosition());
      hasInitializedPosition = true;
    }
  }
}