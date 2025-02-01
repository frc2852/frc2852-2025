package frc.robot.subsystems;

// double solenoid 
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Channel: 0,1
climberUp
climberDown  */
public class climbSubsystem extends SubsystemBase {

    private DoubleSolenoid mClimDoubleSolenoid;

    public climbSubsystem() {

        mClimDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void climberUp() {
        mClimDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void climberDown() {
        mClimDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

}
