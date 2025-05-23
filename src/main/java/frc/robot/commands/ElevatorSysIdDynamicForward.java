package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.MotorSetPoint;

public class ElevatorSysIdDynamicForward extends Command {
    private final Command m_generatedSysIdCommand;
    private final Elevator m_elevator;

    public ElevatorSysIdDynamicForward(Elevator elevator) {
        this.m_elevator = elevator;
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(), // Using default config
                new SysIdRoutine.Mechanism(
                        (voltage) -> elevator.setVoltage(voltage.doubleValue()),
                        null, // Log consumer is null, SysIdRoutine handles logging
                        elevator // Subsystem requirement for SysIdRoutine
                )
        );
        // Generate the specific test command
        m_generatedSysIdCommand = sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
        addRequirements(elevator); // Add requirement for this wrapper command
    }

    @Override
    public void initialize() {
        m_generatedSysIdCommand.initialize();
    }

    @Override
    public void execute() {
        double currentPosition = m_elevator.getPosition();
        if (currentPosition >= MotorSetPoint.ELEVATOR_MAX_HEIGHT) {
            m_elevator.setVoltage(0); // Ensure motor is stopped FIRST
            System.out.println("SysId Forward Test: Upper limit reached. Command will be cancelled.");
            cancel(); // Cancels this wrapper command
        } else {
            m_generatedSysIdCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_generatedSysIdCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_generatedSysIdCommand.isFinished();
    }
}
