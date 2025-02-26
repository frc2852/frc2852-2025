// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.MotorSetPoint;
import frc.robot.subsystems.Climb;

public class ClimbUp extends SequentialCommandGroup {

  public ClimbUp(Climb climb) {
    addCommands(
        new InstantCommand(() -> climb.goToPosition(MotorSetPoint.CLIMBER_CLIMBING_POSITION), climb));
  }
}
