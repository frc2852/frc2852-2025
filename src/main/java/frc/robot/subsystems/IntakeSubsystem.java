// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;
  private final SparkFlexConfig config;

  private double targetSpeed;

  public IntakeSubsystem() {
    motor = new SparkFlex(CanbusId.INTAKE_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
    encoder = motor.getEncoder();

    // Configure motor properties
    config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);

    // Configure encoder conversion factors
    config.encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);

    config.closedLoop.maxMotion
        .maxVelocity(0)
        .maxAcceleration(0)
        .allowedClosedLoopError(2);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntake() {
    targetSpeed = MotorSetPoint.INTAKE_VELOCITY;
     controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  public void reverseIntake() {
    targetSpeed = MotorSetPoint.REVERSE_INTAKE_VELOCITY;
    controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  public void stopIntake(){
    targetSpeed = MotorSetPoint.STOP_INTAKE;
    controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void periodic() {
  }
}
