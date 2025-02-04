// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class ReefAlgeaLevel4 extends SequentialCommandGroup {
  public ReefScoreLevel1(Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
      new InstantCommand((->climb.climberDown(), climb))
    );
  }
}