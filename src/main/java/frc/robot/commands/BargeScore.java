// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.commands.intake.IntakeScoreAlgae;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class BargeScore extends SequentialCommandGroup {
  // change the reefscore 1 to barge
  public BargeScore(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_BARGE), elevator),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_BARGE), wrist),
            new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_BARGE), arm)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition()),
        new IntakeScoreAlgae(intake),
        new MechDrivePosition(elevator, arm, wrist));
  }
}
