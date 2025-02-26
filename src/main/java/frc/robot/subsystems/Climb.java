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

public class Climb extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController controller;

  private final RelativeEncoder encoder;

  private final double P = 0.4;
  private final double I = 0;
  private final double D = 0;

  private double targetPosition;
  private double manualPosition = 0;

  public Climb() {
    motor = new SparkFlex(CanbusId.CLIMBER_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Configure motor properties
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);

    // Configure encoder conversion factors
    motorConfig.encoder
        .positionConversionFactor(MotorSetPoint.CLIMBER_POSITION_CONVERTION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.CLIMBER_VELOCITY_CONVERTION_FACTOR);

    // Configure PID
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(P)
        .i(I)
        .d(D)
        .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
        .maxVelocity(MotorSetPoint.CLIMBER_MAX_VELOCITY)
        .maxAcceleration(MotorSetPoint.CLIMBER_MAX_ACCELERATION)
        .allowedClosedLoopError(MotorSetPoint.CLIMBER_ALLOWED_CLOSED_LOOP_ERROR);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("ClimbManualPosition", manualPosition);
    }
  }

  public void goToPosition(double position) {
    targetPosition = position;
    controller.setReference(targetPosition, ControlType.kPosition);
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - targetPosition) <= 2;
  }

  @Override
  public void periodic() {
    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("ClimbPosition", encoder.getPosition());

      SmartDashboard.putNumber("ClimbCurrent", motor.getOutputCurrent());
      SmartDashboard.putNumber("ClimbTemperature", motor.getMotorTemperature());
      SmartDashboard.putBoolean("ClimbAtPosition", isAtPosition());

      manualPosition = SmartDashboard.getNumber("ClimbManualPosition", manualPosition);
      goToPosition(manualPosition);
    }
  }
}
