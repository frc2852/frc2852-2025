package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAlgae extends Command {
  private final Intake intake;

  public IntakeAlgae(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.intakeAlgae();
  }

  @Override
  public void execute() {
    // No additional looping logic needed.
  }

  @Override
  public boolean isFinished() {
    // Finish if algae is detected.
    return intake.hasAlgae();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}

