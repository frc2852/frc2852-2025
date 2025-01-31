package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class elevator extends SubsystemBase{

    private final SparkFlex motor;
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;
    private final SparkFlexConfig config;
  
    private double targetPosition;

    /* goToBottom
    gotToReefLevel1
    gotToReefLevel2
    gotToReefLevel3
    gotToReefLevel4
    gotToHPStation
    goToBarge
    goToAlgae1-3 */  
    public elevator(){
        motor= new SparkFlex(12, MotorType.kBrushless);
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

    public void goToBottom(){
        targetPosition = 0;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }
    
    public void gotToReefLevel1(){
        targetPosition = 1;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }

    public void gotToReefLevel2(){
        targetPosition = 2;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }

    public void gotToReefLevel3(){
        targetPosition = 3;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }

    public void gotToReefLevel4(){
        targetPosition = 4;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }

    public void goToAlgae1(){
        targetPosition = 5;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }
    
    public void goToBarge(){
        targetPosition = 6;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }

    public void gotToHPStation(){
        targetPosition = 7;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }

    public void goToAlgae2(){
        targetPosition = 8;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    }

    public void goToAlgae3(){
        targetPosition = 9;
        controller.setReference(targetPosition, ControlType.kMAXMotionVelocityControl);
    
    }
    public boolean isAtPosition(){
        return Math.abs(encoder.getPosition() - targetPosition) <= 2; //MARGIN OF ERROR
      }
}

