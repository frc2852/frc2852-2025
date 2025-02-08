// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
<<<<<<< HEAD
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;



public class CageClimb extends SequentialCommandGroup {
  public CageClimb(Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
      new InstantCommand(()->climb.climberDown(),climb)
=======



public class ReefAlgeaLevel4 extends SequentialCommandGroup {
  public ReefScoreLevel1(Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
      new InstantCommand((->climb.climberDown(), climb))
>>>>>>> 9933332777cb0743fe56171357a666b477cb1ca6
    );
  }
}