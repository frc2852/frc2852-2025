// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class Arm extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;
  private final SparkFlexConfig motorConfig;
  
  private double targetPosition;
  private double p;
  private double i;
  private double d;
  private double manualPosition;

  /** Creates a new WristSubsystem. */
  public Arm() {
    motor = new SparkFlex(CanbusId.ARM_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
    encoder = motor.getExternalEncoder();
    encoder.setPosition(0);

    // Configure motor properties
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);

    // Configure encoder conversion factors
    motorConfig.encoder
        .positionConversionFactor(MotorSetPoint.ARM_POSITION_CONVERTION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.ARM_VELOCITY_CONVERTION_FACTOR);

    // Configure PID
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(p)
        .i(i)
        .d(d)
        .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
        .maxVelocity(MotorSetPoint.ARM_MAX_VELOCITY)
        .maxAcceleration(MotorSetPoint.ARM_MAX_ACCELERATION)
        .allowedClosedLoopError(MotorSetPoint.ARM_ALLOWED_CLOSED_LOOP_ERROR);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Arm/P", p);
    SmartDashboard.putNumber("Arm/I", i);
    SmartDashboard.putNumber("Arm/D", d);
    SmartDashboard.putNumber("Arm/ManualPosition", manualPosition);
  }

  public void goToBottom() {
    targetPosition = MotorSetPoint.ARM_BOTTOM_POSITION;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToReefLevel1() {
    targetPosition = MotorSetPoint.ARM_REEF_LEVEL_1;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToReefLevel2() {
    targetPosition = MotorSetPoint.ARM_REEF_LEVEL_2;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToReefLevel3() {
    targetPosition = MotorSetPoint.ARM_REEF_LEVEL_3;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToReefLevel4() {
    targetPosition = MotorSetPoint.ARM_REEF_LEVEL_4;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToBarge() {
    targetPosition = MotorSetPoint.ARM_BARGE;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void gotToHPStation() {
    targetPosition = MotorSetPoint.ARM_HP_STATION;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToAlgae1() {
    targetPosition = MotorSetPoint.ARM_ALGEA_LEVEL_1;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToAlgae2() {
    targetPosition = MotorSetPoint.ARM_ALGEA_LEVEL_2;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void goToAlgae3() {
    targetPosition = MotorSetPoint.ARM_ALGEA_LEVEL_3;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);

  }

  public void goToAlgae4() {
    targetPosition = MotorSetPoint.ARM_ALGEA_LEVEL_4;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);

  }

  public void goToProcessor() {
    targetPosition = MotorSetPoint.ARM_PROCESSOR;
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
      double newP = SmartDashboard.getNumber("Arm/P", p);
      double newI = SmartDashboard.getNumber("Arm/I", i);
      double newD = SmartDashboard.getNumber("Arm/D", d);

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
      double newManualPosition = SmartDashboard.getNumber("Arm/ManualPosition", manualPosition);
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