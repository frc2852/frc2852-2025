// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class Intake extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController controller;

  private DigitalInput coralBeamBreak = new DigitalInput(1);
  private DigitalInput algeaBeamBreak = new DigitalInput(0);

  private double targetSpeed;

  private double p = 0.0001;
  private double i = 0;
  private double d = 0;

  public Intake() {
    motor = new SparkFlex(CanbusId.INTAKE_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure motor properties
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    // Configure encoder conversion factors
    motorConfig.encoder
        .positionConversionFactor(MotorSetPoint.INTAKE_POSITION_CONVERTION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.INTAKE_VELOCITY_CONVERTION_FACTOR);
        

    // Configure PID
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(p)
        .i(i)
        .d(d)
        .velocityFF(1.0 / 5767)
        .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
        .maxVelocity(MotorSetPoint.INTAKE_MAX_VELOCITY)
        .maxAcceleration(MotorSetPoint.INTAKE_MAX_ACCELERATION)
        .allowedClosedLoopError(MotorSetPoint.INTAKE_ALLOWED_CLOSED_LOOP_ERROR);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Runs the intake to collect coral.
   */
  public void intakeCoral() {
    targetSpeed = MotorSetPoint.INTAKE_VELOCITY_CORAL;
    controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  /**
   * Runs the intake to collect algae.
   */
  public void intakeAlgae() {
    targetSpeed = MotorSetPoint.INTAKE_VELOCITY_ALGAE;
    controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  /**
   * Reverses the intake to eject coral.
   */
  public void reverseCoral() {
    targetSpeed = MotorSetPoint.INTAKE_VELOCITY_REVERSE_CORAL;
    controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  /**
   * Reverses the intake to eject algae.
   */
  public void reverseAlgae() {
    targetSpeed = MotorSetPoint.INTAKE_VELOCITY_REVERSE_ALGAE;
    controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  /**
   * Stops the intake.
   */
  public void stop() {
    targetSpeed = MotorSetPoint.STOP_INTAKE;
    controller.setReference(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  public boolean hasGamePiece() {
    return hasAlgae() || hasCoral();
  }

  public boolean hasCoral() {
    return !coralBeamBreak.get();
  }

  public boolean hasAlgae() {
    return !algeaBeamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral", hasCoral());
    SmartDashboard.putBoolean("Algae", hasAlgae());
  }
}
