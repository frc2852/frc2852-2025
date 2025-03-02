// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ReefScoreLevel4 extends SequentialCommandGroup {
  /*
   * Move Elevator
   * Set wrist position
   * Validate we are correct position
   * Score the coral (reverse intake)
   * Set elevator and wrist back to drive position
   */
  public ReefScoreLevel4(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_REEF_LEVEL_4), elevator),
            new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_REEF_LEVEL_4)),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_SCORE_POSITION), wrist)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition() && arm.isAtPosition()),
        new InstantCommand(() -> intake.reverseCoral(), intake),
        new WaitCommand(0.5),
        new InstantCommand(() -> intake.stop(), intake),
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_DRIVE_POSITION), elevator),
            new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_DRIVE_POSITION), arm),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_DRIVE_POSITION), wrist)));
  }
}