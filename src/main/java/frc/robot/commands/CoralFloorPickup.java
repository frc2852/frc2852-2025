package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class CoralFloorPickup extends SequentialCommandGroup {

  // Run in parallel with the drive command
  public CoralFloorPickup(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            // Should already be here
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_DRIVE_POSITION), elevator),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_SCORE_POSITION), wrist),
            new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_FLOOR_PICKUP), arm)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition() && arm.isAtPosition()),
        new InstantCommand(() -> intake.intakeCoral(), intake),
        new WaitUntilCommand(() -> intake.hasGamePiece()),
        new InstantCommand(() -> intake.hold(), intake));
  }
}
