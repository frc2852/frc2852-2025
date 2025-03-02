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

public class Wrist extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController controller;

  private final RelativeEncoder encoder;

  private final double P = 0.1;
  private final double I = 0;
  private final double D = 0;
  private final double OUTPUT_RANGE = 1;

  private double targetPosition;
  private double manualPosition = 0;

  public Wrist() {
    motor = new SparkFlex(CanbusId.WRIST_MOTOR, MotorType.kBrushless);
    controller = motor.getClosedLoopController();

    // Configure encoders
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set encoder position
    motorConfig = new SparkFlexConfig();

    // Configure motor properties
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    // motorConfig.smartCurrentLimit(40);

    motorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    // Configure closed loop PID using the relative encoder (now in degrees).
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P)
        .i(I)
        .d(D)
        .outputRange(-OUTPUT_RANGE, OUTPUT_RANGE);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // if (DriverStation.isTest()) {
    //   SmartDashboard.putNumber("WristManualPosition", manualPosition);
    // }
  }

  /**
   * Command the wrist to go to a specific position.
   *
   * @param position The target wrist angle in degrees.
   */
  public void goToPosition(double position) {
    if(position > 5){
      position = 5;
    } else if (position < 0){
      position = 0;
    }

    targetPosition = position;
    controller.setReference(targetPosition, ControlType.kPosition);
  }

  public boolean isAtPosition() {
    return Math.abs(encoder.getPosition() - targetPosition) <= 1;
  }

  @Override
  public void periodic() {
    // if (DriverStation.isTest()) {
    //   SmartDashboard.putNumber("WristPosition", encoder.getPosition());

    //   SmartDashboard.putNumber("WristCurrent", motor.getOutputCurrent());
    //   SmartDashboard.putNumber("WristTemperature", motor.getMotorTemperature());
    //   SmartDashboard.putBoolean("WristAtPosition", isAtPosition());

    //   manualPosition = SmartDashboard.getNumber("WristManualPosition", manualPosition);
    //   goToPosition(manualPosition);
    // }
  }
}