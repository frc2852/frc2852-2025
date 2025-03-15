package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ReefScoreLevel2Manual extends SequentialCommandGroup {
  public ReefScoreLevel2Manual(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_REEF_LEVEL_2), elevator),
        new WaitUntilCommand(() -> elevator.isAtPosition()),
        new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_REEF_WAIT), arm),
        new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_SCORE_POSITION), wrist),
        new WaitUntilCommand(() -> arm.isAtPosition()));
  }
}
