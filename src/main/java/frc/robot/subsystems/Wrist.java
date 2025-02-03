// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class Wrist extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;
  private final SparkFlexConfig config;

  private double targetPosition;
  private double p;
  private double i;
  private double d;
  private double manualPosition;

  /** Creates a new WristSubsystem. */
  public Wrist() {
    motor = new SparkFlex(CanbusId.WRIST_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
    encoder = motor.getExternalEncoder();
    encoder.setPosition(0);

    // Configure motor properties
    config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);

    // Configure encoder conversion factors
    config.encoder
        .positionConversionFactor(MotorSetPoint.WRIST_POSITION_CONVERTION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.WRIST_VELOCITY_CONVERTION_FACTOR);

    config.closedLoop.maxMotion
        .maxVelocity(MotorSetPoint.WRIST_MAX_VELOCITY)
        .maxAcceleration(MotorSetPoint.WRIST_MAX_ACCELERATION)
        .allowedClosedLoopError(MotorSetPoint.WRIST_ALLOWED_CLOSED_LOOP_ERROR);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Wrist/P", p);
    SmartDashboard.putNumber("Wrist/I", i);
    SmartDashboard.putNumber("Wrist/D", d);
    SmartDashboard.putNumber("Wrist/ManualPosition", manualPosition);
  }

  public void goToBottom() {
    targetPosition = MotorSetPoint.BOTTOM_POSITION;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void gotToReefLevel1() {
    targetPosition = MotorSetPoint.WRIST_REEF_LEVEL_1;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void gotToReefLevel2() {
    targetPosition = MotorSetPoint.WRIST_REEF_LEVEL_2;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void gotToReefLevel3() {
    targetPosition = MotorSetPoint.WRIST_REEF_LEVEL_3;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void gotToReefLevel4() {
    targetPosition = MotorSetPoint.WRIST_REEF_LEVEL_4;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToBarge() {
    targetPosition = MotorSetPoint.WRIST_BARGE;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void gotToHPStation() {
    targetPosition = MotorSetPoint.WRIST_HP_STATION;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToAlgae1() {
    targetPosition = MotorSetPoint.WRIST_ALGEA_LEVEL_1;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToAlgae2() {
    targetPosition = MotorSetPoint.WRIST_ALGEA_LEVEL_2;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToAlgae3() {
    targetPosition = MotorSetPoint.WRIST_ALGEA_LEVEL_3;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);

  }

  public void goToAlgae4() {
    targetPosition = MotorSetPoint.WRIST_ALGEA_LEVEL_4;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);

  }


  public boolean isAtPosition() {
    double encoderPosition = encoder.getPosition();
    return Math.abs(encoderPosition - targetPosition) <= 2; // MARGIN OF ERROR
  }

  @Override
  public void periodic() {
    if (DriverStation.isTest()) {
      // Read PID coefficients from SmartDashboard
      double newP = SmartDashboard.getNumber("Wrist/P", p);
      double newI = SmartDashboard.getNumber("Wrist/I", i);
      double newD = SmartDashboard.getNumber("Wrist/D", d);

      // Update PID values if they have changed
      if (newP != p || newI != i || newD != d) {
        p = newP;
        i = newI;
        d = newD;

        // Update closed loop PID settings
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(p)
            .i(i)
            .d(d)
            .outputRange(-1, 1);

        // Reapply the updated configuration
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

      // Read manual position from SmartDashboard
      double newManualPosition = SmartDashboard.getNumber("Wrist/ManualPosition", manualPosition);
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
}