// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.ReefScoreLevel1;
import frc.robot.commands.ReefScoreLevel2;
import frc.robot.commands.ReefScoreLevel3;
import frc.robot.commands.ReefScoreLevel4;
import frc.robot.commands.ReefAlgeaLevel1;
import frc.robot.commands.ReefAlgeaLevel2;
import frc.robot.commands.ReefAlgeaLevel3;
import frc.robot.commands.ReefAlgeaLevel4;

import frc.robot.commands.BargeScore;
import frc.robot.commands.CoralPickUPPosition;
import frc.robot.commands.HSPickUp;
import frc.robot.commands.CageClimb;
import frc.robot.commands.ProcessorScore;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // how to import the other class -_-
  private final Climb climb = new Climb();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake();

  // Commands
  private final BargeScore bargeScore = new BargeScore(elevator, wrist, intake);
  private final ProcessorScore processorScore = new ProcessorScore(wrist, intake, elevator);
  private final CageClimb cageClimb = new CageClimb(climb);
  private final HSPickUp hsPickUp = new HSPickUp(elevator, wrist, intake);
  private final CoralPickUPPosition coralPickUpPosition = new CoralPickUPPosition(elevator, wrist, intake);

  private final ReefScoreLevel1 reefScoreLevel1 = new ReefScoreLevel1(elevator, wrist, intake);
  private final ReefScoreLevel2 reefScoreLevel2 = new ReefScoreLevel2(elevator, wrist, intake);
  private final ReefScoreLevel3 reefScoreLevel3 = new ReefScoreLevel3(elevator, wrist, intake);
  private final ReefScoreLevel4 reefScoreLevel4 = new ReefScoreLevel4(elevator, wrist, intake);

  private final ReefAlgeaLevel1 reefAlgeaLevel1 = new ReefAlgeaLevel1(elevator, wrist, intake);
  private final ReefAlgeaLevel2 reefAlgeaLevel2 = new ReefAlgeaLevel2(elevator, wrist, intake);
  private final ReefAlgeaLevel3 reefAlgeaLevel3 = new ReefAlgeaLevel3(elevator, wrist, intake);
  private final ReefAlgeaLevel4 reefAlgeaLevel4 = new ReefAlgeaLevel4(elevator, wrist, intake);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    if (DriverStation.isTest()) {

      driverController.a().onTrue(reefScoreLevel1);
      driverController.b().onTrue(reefScoreLevel2);
      driverController.x().onTrue(reefScoreLevel3);
      driverController.y().onTrue(reefScoreLevel4);
      driverController.leftBumper().onTrue(bargeScore);
      driverController.rightBumper().onTrue(processorScore);

      operatorController.a().onTrue(reefAlgeaLevel1);
      operatorController.b().onTrue(reefAlgeaLevel2);
      operatorController.x().onTrue(reefAlgeaLevel3);
      operatorController.y().onTrue(reefAlgeaLevel4);
      operatorController.leftBumper().onTrue(coralPickUpPosition);
      operatorController.rightBumper().onTrue(hsPickUp);
      operatorController.leftTrigger().onTrue(cageClimb);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() {
   * // An example command will be run in autonomous
   * return Autos.exampleAuto(null);
   * }
   */
}
