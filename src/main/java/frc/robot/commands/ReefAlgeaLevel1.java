// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAlgeaLevel1 extends SequentialCommandGroup {
  public ReefAlgeaLevel1(Elevator elevator,Wrist wrist,Intake intake) {
 addComands(
  new ParallelCommandGroup(
  new InstantCommand(()->elevator.goToAlgea1(), elevator) 
  new InstantCommand(()->wrist.goToAlgea1(), wrist))
  new waitUntilCommand(()->elevator.isAtPosition() && wrist.isAtPosition())
  new waitCommand(2)
  
  );
  }
}
