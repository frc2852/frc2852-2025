// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class Wrist extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController controller;

  private final AbsoluteEncoder absEncoder;
  private final RelativeEncoder encoder;

  private double targetPosition;
  private double manualPosition;

  private double p = 0.4;
  private double i = 0;
  private double d = 0;

  /** Creates a new WristSubsystem. */
  public Wrist() {
    motor = new SparkFlex(CanbusId.WRIST_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
    absEncoder = motor.getAbsoluteEncoder();

    encoder = motor.getEncoder();

    // Configure motor properties
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kCoast);
    motorConfig.inverted(false);

    // Configure encoder conversion factors
    motorConfig.absoluteEncoder
        .positionConversionFactor(180)
        .velocityConversionFactor(1)
        .inverted(true);

    // Configure PID
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(p)
        .i(i)
        .d(d)
        .outputRange(-1, 1);

    encoder.setPosition(absEncoder.getPosition());

    motorConfig.closedLoop.maxMotion
        .maxVelocity(MotorSetPoint.WRIST_MAX_VELOCITY)
        .maxAcceleration(MotorSetPoint.WRIST_MAX_ACCELERATION)
        .allowedClosedLoopError(MotorSetPoint.WRIST_ALLOWED_CLOSED_LOOP_ERROR);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("WristManualPosition", 90);
    SmartDashboard.putNumber("WristP", p);
    SmartDashboard.putNumber("WristI", i);
    SmartDashboard.putNumber("WristD", d);
  }

  public void goToPosition(double position) {
    double arbFeedForward = 0;
    targetPosition = position;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, arbFeedForward);
  }

  public boolean isAtPosition() {
    double encoderPosition = encoder.getPosition();
    return Math.abs(encoderPosition - targetPosition) <= 1; // MARGIN OF ERROR
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("WristAbsPosition", absEncoder.getPosition());
    SmartDashboard.putNumber("WristPosition", encoder.getPosition());

    manualPosition = SmartDashboard.getNumber("WristManualPosition", 90);
    goToPosition(manualPosition);

    // Read PID coefficients from SmartDashboard
    double newP = SmartDashboard.getNumber("WristP", p);
    double newI = SmartDashboard.getNumber("WristI", i);
    double newD = SmartDashboard.getNumber("WristD", d);

    // Update PID values if they have changed
    if (newP != p || newI != i || newD != d) {
      p = newP;
      i = newI;
      d = newD;

      // Update closed loop PID settings
      motorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(p)
          .i(i)
          .d(d)
          .outputRange(-1, 1);

      // Reapply the updated configuration
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Read manual position from SmartDashboard
    double newManualPosition = SmartDashboard.getNumber("WristManualPosition", manualPosition);
    // If the manual position has been changed, update the setpoint
    if (newManualPosition != manualPosition) {
      manualPosition = newManualPosition;
      targetPosition = manualPosition;
      controller.setReference(
          targetPosition,
          ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    }
  }
}