package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.commands.intake.IntakeScoreCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ReefScoreLevel2 extends SequentialCommandGroup {
  public ReefScoreLevel2(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_REEF_LEVEL_2), elevator),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_SCORE_POSITION), wrist)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition()),
        new IntakeScoreCoral(intake),
        new MechDrivePosition(elevator, arm, wrist));
  }
}
