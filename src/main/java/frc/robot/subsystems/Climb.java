package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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

public class Climb extends SubsystemBase {
  

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController controller;

  private final AbsoluteEncoder absEncoder;

  private final double P = 0.3;
  
  private final double I = 0;
  private final double D = 0;

  private double targetPosition;

  private double manualPosition = 9.5;

  public Climb() {
    
    motor = new SparkFlex(CanbusId.CLIMBER_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoders
    absEncoder = motor.getAbsoluteEncoder();

    // Configure motor properties
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    // motorConfig.smartCurrentLimit(10);

    // Configure encoder conversion factors
    motorConfig.absoluteEncoder
        .positionConversionFactor(MotorSetPoint.CLIMBER_POSITION_CONVERTION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.CLIMBER_VELOCITY_CONVERTION_FACTOR)
        .inverted(true);

    // Configure PID
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(P)
        .i(I)
        .d(D)
        .outputRange(-1, 1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    goToPosition(MotorSetPoint.CLIMBER_DRIVE_POSITION);
    SmartDashboard.putNumber("ManualPosition", manualPosition);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public void goToPosition(double position) {
    targetPosition = position;
    controller.setReference(targetPosition, ControlType.kPosition);
  }

  public boolean isAtPosition() {
    return Math.abs(absEncoder.getPosition() - targetPosition) <= 0.2;
  }

  @Override
  public void periodic() {
    // if(!hasInitialized && absEncoder.getPosition() > 1) {
    //   encoder.setPosition(absEncoder.getPosition());
    //   hasInitialized = true;
    // }
    SmartDashboard.putNumber("ClimbAbsEncoder", absEncoder.getPosition()); 
    


    // manualPosition = SmartDashboard.getNumber("ManualPosition", manualPosition);
    // if(manualPosition != targetPosition){
    //   goToPosition(manualPosition);
    // }
  }
}
