// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  public ReefScoreLevel4(Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(new ParallelCommandGroup(
        new InstantCommand(() -> elevator.gotToReefLevel4(), elevator),
        new InstantCommand(() -> wrist.goToReefLevel4(), wrist)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition()),
        new InstantCommand(() -> intake.reverseIntake()),
        new WaitCommand(2),
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToBottom(), elevator),
            new InstantCommand(() -> wrist.goToBottom(), wrist)));
  }
}