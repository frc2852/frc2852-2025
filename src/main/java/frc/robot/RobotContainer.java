package frc.robot;

import frc.robot.Constants.MotorSetPoint;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Pattern;
import frc.robot.Constants.ScoringLevel;
import frc.robot.commands.BargeScore;
import frc.robot.commands.ClimberGrabPosition;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.CoralFloorPickup;
import frc.robot.commands.DrivePosition;
import frc.robot.commands.IntakeStationPickup;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.ReefAlgeaLevel1;
import frc.robot.commands.ReefAlgeaLevel1Waste;
import frc.robot.commands.ReefAlgeaLevel2;
import frc.robot.commands.ReefAlgeaLevel2Waste;
import frc.robot.commands.ReefScoreLevel1;
import frc.robot.commands.ReefScoreLevel2;
import frc.robot.commands.ReefScoreLevel3;
import frc.robot.commands.ReefScoreLevel4;
import frc.robot.commands.intake.IntakeAlgae;
import frc.robot.commands.intake.IntakeCoral;
import frc.robot.commands.intake.IntakeScoreAlgae;
import frc.robot.commands.intake.IntakeScoreCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER);

  // Subsystem
  private final Swerve drivebase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/pinky"));
  private final Climb climb = new Climb();
  private final Elevator elevator = new Elevator(); // done
  private final Intake intake = new Intake(); // done
  private final LED led = new LED(); // done
  private final Arm arm = new Arm(); // done
  private final Wrist wrist = new Wrist();

  // Debug only
  private final IntakeAlgae intakeAlgae = new IntakeAlgae(intake);
  private final IntakeCoral intakeCoral = new IntakeCoral(intake);
  private final IntakeScoreAlgae intakeScoreAlgae = new IntakeScoreAlgae(intake);
  private final IntakeScoreCoral intakeScoreCoral = new IntakeScoreCoral(intake);
  // End Debug

  // Commands
  private final BargeScore bargeScore = new BargeScore(elevator, arm, wrist, intake);
  private final IntakeStationPickup intakeStationPickup = new IntakeStationPickup(elevator, arm, wrist, intake);

  private final ReefAlgeaLevel1 reefAlgeaLevel1 = new ReefAlgeaLevel1(elevator, arm, wrist, intake);
  private final ReefAlgeaLevel2 reefAlgeaLevel2 = new ReefAlgeaLevel2(elevator, arm, wrist, intake);
  private final ReefAlgeaLevel1Waste reefAlgeaLevel1Waste = new ReefAlgeaLevel1Waste(elevator, arm, wrist, intake);
  private final ReefAlgeaLevel2Waste reefAlgeaLevel2Waste = new ReefAlgeaLevel2Waste(elevator, arm, wrist, intake);

  private final ReefScoreLevel1 reefScoreLevel1 = new ReefScoreLevel1(elevator, arm, wrist, intake);
  private final ReefScoreLevel2 reefScoreLevel2 = new ReefScoreLevel2(elevator, arm, wrist, intake);
  private final ReefScoreLevel3 reefScoreLevel3 = new ReefScoreLevel3(elevator, arm, wrist, intake);
  private final ReefScoreLevel4 reefScoreLevel4 = new ReefScoreLevel4(elevator, arm, wrist, intake);

  private final ProcessorScore processorScore = new ProcessorScore(elevator, arm, wrist, intake);
  private final DrivePosition drivePosition = new DrivePosition(elevator, arm, wrist, intake);

  // private final ClimberDrivePosition climbDrivePosition = new
  private final ClimberGrabPosition climberGrabPosition = new ClimberGrabPosition(climb);
  private final ClimberUp climberUp = new ClimberUp(climb);

  // Auto only
  private final CoralFloorPickup coralFloorPickup = new CoralFloorPickup(elevator, arm, wrist, intake);

  // Store the currently scheduled scoring command
  private Command aButtonCommand;

  // Drive commands
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverController.getLeftY() * 1,
      () -> driverController.getLeftX() * 1)
      .withControllerRotationAxis(() -> driverController.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // Auto
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Start data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DriverStation.silenceJoystickConnectionWarning(true);

    // Port forwarding
    PortForwarder.add(5800, "barge-photon.local", 5800);
    PortForwarder.add(5800, "pickup-station-photon.local", 5800);
    PortForwarder.add(5800, "reef-photon.local", 5800);

    // Set default led color during initialization
    led.setPattern(Pattern.OFF);

    configureBindings();
  }

  private void configureBindings() {
    configureDriverBindings();
    configureOperatorBindings();
    configurePathPlanner();
    configureLEDTriggers();
  }

  private void configureDriverBindings() {
    // Drive commands
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    if (Robot.isSimulation()) {
      driverController.start()
          .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    // driverController.b().onTrue(
    //     new InstantCommand(() -> {
    //       Command newCommand = drivebase.driveToPose(RobotControlState.getZonePose());
    //       if (newCommand != null) {
    //         newCommand.schedule();
    //       }
    //     }));

    driverController.leftBumper().and(driverController.rightBumper())
        .onTrue(new ParallelCommandGroup(
            // new MechClimbPosition(elevator, arm, wrist),
            new InstantCommand(() -> RobotControlState.toggleClimb())));

    driverController.b().onTrue(new InstantCommand(() -> {
      if (aButtonCommand != null && aButtonCommand.isScheduled()) {
        aButtonCommand.cancel();
        drivePosition.schedule();
      }
    }));

    driverController.a().onTrue(new InstantCommand(() -> {
      // Cancel the previous command if it's still running
      if (aButtonCommand != null && aButtonCommand.isScheduled()) {
        aButtonCommand.cancel();
      }
      // Retrieve and schedule the new command
      aButtonCommand = getSelectedCommand();
      if (aButtonCommand != null) {
        aButtonCommand.schedule();
      }
    }));
  }

  public Command getSelectedCommand() {
    boolean hasAlgae = intake.hasAlgae();
    boolean hasGamePiece = intake.hasGamePiece();

    // Climb mode check.
    if (RobotControlState.isClimbEnabled()) {
      if (climb.getTargetPosition() == MotorSetPoint.CLIMBER_GRAB_POSITION) {
        return climberUp;
      } else {
        return climberGrabPosition;
      }
    }

    // Early exit: if nothing is picked up and we're not in algae mode.
    if (!hasGamePiece && !RobotControlState.isAlgaeMode()) {
      return intakeStationPickup;
    }

    // Barge and Processor scoring share similar behavior when !hasAlgae.
    if (RobotControlState.isBargeScore() || RobotControlState.isProcessorScore()) {
      if (hasAlgae) {
        return RobotControlState.isBargeScore() ? bargeScore : processorScore;
      } else {
        return getAlgaeReefPositionCommand();
      }
    }

    // Handle algae waste separately.
    if (RobotControlState.isAlgaeWaste()) {
      return getAlgaeWasteScoringCommand();
    }

    // Default scoring command.
    return getCoralReefCommand();
  }

  private Command getAlgaeReefPositionCommand() {
    switch (RobotControlState.getScoringLevel()) {
      case LEVEL_1:
        return reefAlgeaLevel1;
      case LEVEL_2:
        return reefAlgeaLevel2;
      default:
        return null;
    }
  }

  private Command getAlgaeWasteScoringCommand() {
    switch (RobotControlState.getScoringLevel()) {
      case LEVEL_1:
        return reefAlgeaLevel1Waste;
      case LEVEL_2:
        return reefAlgeaLevel2Waste;
      default:
        return null;
    }
  }

  private Command getCoralReefCommand() {
    switch (RobotControlState.getScoringLevel()) {
      case LEVEL_1:
        return reefScoreLevel1;
      case LEVEL_2:
        return reefScoreLevel2;
      case LEVEL_3:
        return reefScoreLevel3;
      case LEVEL_4:
        return reefScoreLevel4;
      default:
        return null;
    }
  }

  private void configureOperatorBindings() {
    // Bind operator buttons to set scoring levels
    operatorController.povDown()
        .onTrue(new InstantCommand(() -> RobotControlState.setScoringLevel(ScoringLevel.LEVEL_1)));

    operatorController.povLeft()
        .onTrue(new InstantCommand(() -> RobotControlState.setScoringLevel(ScoringLevel.LEVEL_2)));

    operatorController.povUp()
        .onTrue(new InstantCommand(() -> RobotControlState.setScoringLevel(ScoringLevel.LEVEL_3)));

    operatorController.povRight()
        .onTrue(new InstantCommand(() -> RobotControlState.setScoringLevel(ScoringLevel.LEVEL_4)));

    operatorController.x()
        .onTrue(new InstantCommand(() -> RobotControlState.setProcessor()));

    operatorController.b()
        .onTrue(new InstantCommand(() -> RobotControlState.setBarge()));

    operatorController.a()
        .onTrue(new InstantCommand(() -> RobotControlState.setAlgaeWaste()));
  }

  private void configureLEDTriggers() {
    Trigger hasGamePiece = new Trigger(() -> intake.hasGamePiece());
    hasGamePiece.onTrue(new InstantCommand(() -> led.activatePattern(Pattern.GREEN)));
    hasGamePiece.onFalse(new InstantCommand(() -> led.deactivatePattern(Pattern.GREEN)));

    Trigger isClimbEnabled = new Trigger(() -> RobotControlState.isClimbEnabled());
    isClimbEnabled.onTrue(new InstantCommand(() -> led.activatePattern(Pattern.BLUE)));
    isClimbEnabled.onFalse(new InstantCommand(() -> led.deactivatePattern(Pattern.BLUE)));
  }

  private void configurePathPlanner() {

    // Register commands
    NamedCommands.registerCommand("intakeAlgae", intakeAlgae);
    NamedCommands.registerCommand("intakeCoral", intakeCoral);
    NamedCommands.registerCommand("intakeScoreAlgae", intakeScoreAlgae);
    NamedCommands.registerCommand("intakeScoreCoral", intakeScoreCoral);
    NamedCommands.registerCommand("bargeScore", bargeScore);

    NamedCommands.registerCommand("coralFloorPickup", coralFloorPickup);
    NamedCommands.registerCommand("intakeStationPickup", intakeStationPickup);

    NamedCommands.registerCommand("processorScore", processorScore);

    NamedCommands.registerCommand("reefAlgeaLevel1", reefAlgeaLevel1);
    NamedCommands.registerCommand("reefAlgeaLevel2", reefAlgeaLevel2);

    NamedCommands.registerCommand("reefScoreLevel1", reefScoreLevel1);
    NamedCommands.registerCommand("reefScoreLevel2", reefScoreLevel2);
    NamedCommands.registerCommand("reefScoreLevel3", reefScoreLevel3);
    NamedCommands.registerCommand("reefScoreLevel4", reefScoreLevel4);

    // Build an auto chooser
    autoChooser = AutoBuilder.buildAutoChooser("Example");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {

  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
