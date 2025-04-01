// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ReefScoreAlignment extends SequentialCommandGroup {
  /*
   * Move Elevator
   * Set wrist position
   * Validate we are correct position
   * Score the coral (reverse intake)
   * Set elevator and wrist back to drive position
   */
  public ReefScoreAlignment(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new InstantCommand(() -> intake.stop(), intake),
        new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_REEF_WAIT), arm),
        new WaitUntilCommand(() -> elevator.isAtPosition() && arm.isAtPosition()));
  }
}