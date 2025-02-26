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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetPoint;

public class Climb extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;
  private final SparkFlexConfig motorConfig;

  private double targetPosition;
  private double p = 0.4;
  private double i = 0;
  private double d = 0;
  private double manualPosition;

  public Climb() {
    motor = new SparkFlex(CanbusId.CLIMBER_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoder
    encoder = motor.getEncoder();

    // Configure motor properties
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);

    // Configure encoder conversion factors
    motorConfig.encoder
        .positionConversionFactor(MotorSetPoint.CLIMBER_POSITION_CONVERTION_FACTOR)
        .velocityConversionFactor(MotorSetPoint.CLIMBER_VELOCITY_CONVERTION_FACTOR);

    // Configure PID
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(p)
        .i(i)
        .d(d)
        .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
        .maxVelocity(MotorSetPoint.CLIMBER_MAX_VELOCITY)
        .maxAcceleration(MotorSetPoint.CLIMBER_MAX_ACCELERATION)
        .allowedClosedLoopError(MotorSetPoint.CLIMBER_ALLOWED_CLOSED_LOOP_ERROR);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Climber/P", p);
    SmartDashboard.putNumber("Climber/I", i);
    SmartDashboard.putNumber("Climber/D", d);
    SmartDashboard.putNumber("Climber/ManualPosition", manualPosition);
  }

  public void climberUp() {
    targetPosition = MotorSetPoint.CLIMBER_CLIMBING_POSITION;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void climberDown() {
    targetPosition = MotorSetPoint.CLIMBER_BOTTOM_POSITION;
    controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

}
