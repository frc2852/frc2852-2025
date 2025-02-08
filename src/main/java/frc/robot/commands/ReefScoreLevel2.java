package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
/*Dani
ReefScoreLevel1
ReefScoreLevel2
ReefScoreLevel3
ReefScoreLevel4
Workflow:
  
Move Elevator
Set wrist position
Validate we are correct position
Score the coral (reverse intake)
Set elevator and wrist back to drive position
 */
public class ReefScoreLevel2 extends SequentialCommandGroup {
  public ReefScoreLevel2(Elevator elevator, Wrist wrist , Intake intake ){
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(()-> elevator.gotToReefLevel2(),elevator),
        new InstantCommand (()-> wrist.goToReefLevel2(),wrist)),
        new WaitUntilCommand(()-> elevator.isAtPosition()&& wrist.isAtPosition()),
        new InstantCommand(()-> intake.reverseIntake()),
        new WaitCommand(2),
        new InstantCommand(()-> intake. stopIntake()),
        new ParallelCommandGroup(
          new InstantCommand(()-> elevator.goToBottom(),elevator),
          new InstantCommand(()-> wrist.goToBottom(),wrist)));
  }
}