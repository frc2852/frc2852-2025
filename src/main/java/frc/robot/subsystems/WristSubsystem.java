// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class WristSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final SparkClosedLoopController controller;
    private final AbsoluteEncoder encoder;
    private final SparkFlexConfig config;
  
    private double targetPosition;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
   motor= new SparkFlex(CanbusId.WRIST_MOTOR,MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
  encoder = motor.getAbsoluteEncoder();
  encoder.setPosition(0);

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


  public void goToBottom(){
      targetPosition = MotorSetPoint.BOTTOM_POSITION;
      controller.setReference(targetPosition, ControlType.kPosition);
  }
  
  public void gotToReefLevel1(){
      targetPosition = MotorSetPoint.REEF_LEVEL_1;
      controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void gotToReefLevel2(){
      targetPosition = MotorSetPoint.REEF_LEVEL_2;
      controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void gotToReefLevel3(){
      targetPosition = MotorSetPoint.REEF_LEVEL_3;
      controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void gotToReefLevel4(){
      targetPosition = MotorSetPoint.REEF_LEVEL_4;
      controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void goToBarge(){
      targetPosition = MotorSetPoint.BARGE;
      controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void gotToHPStation(){
      targetPosition = MotorSetPoint.HP_STATION;
      controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void goToAlgae1(){
    targetPosition = MotorSetPoint.ALGEA_LEVEL_1;
    controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void goToAlgae2(){
      targetPosition = MotorSetPoint.ALGEA_LEVEL_2;
      controller.setReference(targetPosition, ControlType.kPosition);
  }

  public void goToAlgae3(){
      targetPosition = MotorSetPoint.ALGEA_LEVEL_3;
      controller.setReference(targetPosition, ControlType.kPosition);
  
  }
  
  public boolean isAtPosition(){
        double encoderPosition = encoder.getPosition();
      return Math.abs(encoderPosition - targetPosition) <= 2; //MARGIN OF ERROR
  }

      
  @Override
  public void periodic() {
    }
}