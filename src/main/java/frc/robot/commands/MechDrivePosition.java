// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class MechDrivePosition extends ParallelCommandGroup {
  public MechDrivePosition(Elevator elevator, Arm arm, Wrist wrist) {
    new InstantCommand(() -> elevator.goToPosition(MotorSetPoint.ELEVATOR_DRIVE_POSITION), elevator);
    new InstantCommand(() -> arm.goToPosition(MotorSetPoint.ARM_DRIVE_POSITION), arm);
    new InstantCommand(() -> wrist.goToPosition(MotorSetPoint.WRIST_DRIVE_POSITION), wrist);
  }
}
