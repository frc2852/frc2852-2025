package frc.robot.utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorFeedforward extends edu.wpi.first.math.controller.ElevatorFeedforward {

    private final double kS;
    private final double kG;
    private final double kV;
    private final double kA;

    /**
     * Creates a new ElevatorFeedforward.
     *
     * @param kS The static friction feedforward gain (volts).
     * @param kG The gravity feedforward gain (volts).
     * @param kV The velocity feedforward gain (volts * seconds / meter).
     * @param kA The acceleration feedforward gain (volts * seconds^2 / meter).
     */
    public ElevatorFeedforward(double kS, double kG, double kV, double kA) {
        super(kS, kG, kV, kA);
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Calculates the feedforward voltage for a given velocity and acceleration.
     *
     * @param velocity The current velocity (meters per second).
     * @param acceleration The desired acceleration (meters per second squared).
     * @return The feedforward voltage (volts).
     */
    public double calculate(double velocity, double acceleration) {
        return super.calculate(velocity, acceleration);
    }

    // It might be useful to add getters for the gains if direct access is needed elsewhere,
    // but for now, the superclass handles the calculation.
}
