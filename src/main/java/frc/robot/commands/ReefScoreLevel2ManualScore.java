// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class ReefScoreLevel2ManualScore extends SequentialCommandGroup {
  public ReefScoreLevel2ManualScore(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_REEF_LEVEL_2_MANUAL)),
        new WaitUntilCommand(() -> arm.isAtPosition()),
        new InstantCommand(() -> intake.reverseCoral(), intake),
        new WaitUntilCommand(() -> !intake.hasGamePiece()),
        new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_DRIVE_POSITION), wrist),
        new WaitUntilCommand(() -> wrist.isAtPosition()),
        new ParallelCommandGroup(
            new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_DRIVE_POSITION), arm),
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_DRIVE_POSITION), elevator),
            new InstantCommand(() -> intake.stop(), intake)));
  }
}