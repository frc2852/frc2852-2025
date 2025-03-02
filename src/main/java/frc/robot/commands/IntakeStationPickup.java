// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.commands.intake.IntakeCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/**
 * CoralPickupPosition
 * Workflow:
 * 
 * Move Elevator
 * Set wrist position
 * Validate we are correct position
 * Intake the Coral (intake)
 * Set elevator and wrist back to drive position
 */
public class IntakeStationPickup extends SequentialCommandGroup {
  public IntakeStationPickup(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_INTAKE_STATION), elevator),
            new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_SCORE_POSITION), wrist),
            new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_INTAKE_STATION), arm)),
        new WaitUntilCommand(() -> elevator.isAtPosition() && wrist.isAtPosition()),
        new IntakeCoral(intake),
        new MechDrivePosition(elevator, arm, wrist));
  }
}
