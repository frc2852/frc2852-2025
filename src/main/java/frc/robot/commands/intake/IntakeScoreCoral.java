package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeScoreCoral extends Command {
  private final Intake intake;

  public IntakeScoreCoral(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.reverseCoral();
  }

  @Override
  public void execute() {
    // No additional looping logic needed.
  }

  @Override
  public boolean isFinished() {
    // Finish if coral is detected.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
