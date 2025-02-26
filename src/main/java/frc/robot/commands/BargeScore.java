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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class BargeScore extends SequentialCommandGroup {
  // change the reefscore 1 to barge
  public BargeScore(Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_BARGE), elevator),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_BARGE), wrist)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition()),
        new InstantCommand(() -> intake.reverseAlgae()),
        new WaitCommand(2),
        new InstantCommand(() -> intake.stop()),
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_DRIVE_POSITION), elevator),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_DRIVE_POSITION), wrist)));
  }
}
