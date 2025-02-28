package frc.robot.subsystems;

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

public class Elevator extends SubsystemBase {

    private final SparkFlex motor;
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;
    private final SparkFlexConfig config;

    private double targetPosition;
    private double p;
    private double i;
    private double d;
    private double manualPosition;

    public Elevator() {
        // configure motor
        motor = new SparkFlex(CanbusId.ELEVATOR_MOTOR, MotorType.kBrushless);
        controller = motor.getClosedLoopController();

        // Configure encoder
        encoder = motor.getEncoder();
        encoder.setPosition(MotorSetPoint.ELEVATOR_POSITION);

        // Configure motor properties
        config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        // Configure encoder conversion factors
        config.encoder
                .positionConversionFactor(MotorSetPoint.ELEVATOR_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(MotorSetPoint.ELEVATOR_VELOCITY_CONVERSION_FACTOR);

        config.closedLoop.maxMotion
                .maxVelocity(MotorSetPoint.ELEVATOR_MAX_VELOCITY)
                .maxAcceleration(MotorSetPoint.ELEVATOR_MAX_ACCELERATION)
                .allowedClosedLoopError(MotorSetPoint.ELEVATOR_CLOSED_LOOP);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Elevator/P", p);
        SmartDashboard.putNumber("Elevator/I", i);
        SmartDashboard.putNumber("Elevator/D", d);
        SmartDashboard.putNumber("Elevator/ManualPosition", manualPosition);
    }

    public void goToBottom() {
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel1() {
        targetPosition = MotorSetPoint.ELEVATOR_REEF_LEVEL_1;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel2() {
        targetPosition = MotorSetPoint.ELEVATOR_REEF_LEVEL_2;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel3() {
        targetPosition = MotorSetPoint.ELEVATOR_REEF_LEVEL_3;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel4() {
        targetPosition = MotorSetPoint.ELEVATOR_REEF_LEVEL_4;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToAlgae1() {
        targetPosition = MotorSetPoint.ELEVATOR_ALGEA_LEVEL_1;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToBarge() {
        targetPosition = MotorSetPoint.ELEVATOR_BARGE;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToHPStation() {
        targetPosition = MotorSetPoint.ELEVATOR_HP;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToAlgae2() {
        targetPosition = MotorSetPoint.ELEVATOR_REEF_LEVEL_2;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToAlgae3() {
        targetPosition = MotorSetPoint.ELEVATOR_ALGEA_LEVEL_3;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);

    }

    public void goToAlgae4() {
        targetPosition = MotorSetPoint.ELEVATOR_ALGEA_LEVEL_4;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);

    }

    public boolean isAtPosition() {
        return Math.abs(encoder.getPosition() - targetPosition) <= 2; // MARGIN OF ERROR
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (DriverStation.isTest()) {
            // Read PID coefficients from SmartDashboard
            double newP = SmartDashboard.getNumber("Elevator/P", p);
            double newI = SmartDashboard.getNumber("Elevator/I", i);
            double newD = SmartDashboard.getNumber("Elevator/D", d);

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
            double newManualPosition = SmartDashboard.getNumber("Elevator/ManualPosition", manualPosition);
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
