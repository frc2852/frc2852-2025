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
import frc.robot.Constants.MotorSetPoint;

public class elevatorSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;
    private final SparkFlexConfig config;

    private double targetPosition;

    public elevatorSubsystem() {
        // configure motor
        motor = new SparkFlex(12, MotorType.kBrushless);
        controller = motor.getClosedLoopController();

        // Configure encoder
        encoder = motor.getEncoder();
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

    @Override
    public void periodic() {

    }

    public void goToBottom() {
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel1() {
        targetPosition = MotorSetPoint.ELEVATATOR_REEF_LEVEL_1;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel2() {
        targetPosition = MotorSetPoint.ELEVATATOR_REEF_LEVEL_2;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel3() {
        targetPosition = MotorSetPoint.ELEVATATOR_REEF_LEVEL_3;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToReefLevel4() {
        targetPosition = MotorSetPoint.ELEVATATOR_REEF_LEVEL_4;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToAlgae1() {
        targetPosition = MotorSetPoint.ELEVATATOR_ALGEA_LEVEL_1;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToBarge() {
        targetPosition = MotorSetPoint.ELEVATATOR_BARGE;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void gotToHPStation() {
        targetPosition = MotorSetPoint.ELEVATATOR_HP;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToAlgae2() {
        targetPosition = MotorSetPoint.ELEVATATOR_REEF_LEVEL_2;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    public void goToAlgae3() {
        targetPosition = MotorSetPoint.ELEVATATOR_ALGEA_LEVEL_3;
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);

    }

    public boolean isAtPosition() {
        return Math.abs(encoder.getPosition() - targetPosition) <= 2; // MARGIN OF ERROR
    }
}
