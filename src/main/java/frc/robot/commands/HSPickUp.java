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

public class HSPickUp extends SequentialCommandGroup {
  public HSPickUp(Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.gotToHPStation(), elevator),
            new InstantCommand(() -> wrist.gotToHPStation(), wrist)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition()),
        new InstantCommand(() -> intake.runIntake()),
        new WaitCommand(2),
        new InstantCommand(() -> intake.stopIntake()),
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToBottom(), elevator),
            new InstantCommand(() -> wrist.goToBottom(), wrist)));
  }
}