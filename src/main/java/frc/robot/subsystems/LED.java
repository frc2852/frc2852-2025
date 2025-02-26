package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Pattern;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.PWM;

public class LED extends SubsystemBase {

  private final PWM blinkin;
  private final List<Pattern> activePatterns = new ArrayList<>();

  public LED() {
    blinkin = new PWM(Constants.PWM.BLINKIN);
  }

  /**
   * Adds a pattern to the stack and updates the LED.
   */
  public void activatePattern(Pattern pattern) {
    if (!activePatterns.contains(pattern)) {
      activePatterns.add(pattern);
      updateLED();
    }
  }

  /**
   * Removes a pattern from the stack and updates the LED.
   */
  public void deactivatePattern(Pattern pattern) {
    activePatterns.remove(pattern);
    updateLED();
  }

  /**
   * Updates the LED based on the most recently activated pattern.
   */
  private void updateLED() {
    if (activePatterns.isEmpty()) {
      setPattern(Pattern.OFF); // Turn off if no active patterns
    } else {
      setPattern(activePatterns.get(activePatterns.size() - 1)); // Most recent pattern
    }
  }

  /**
   * Sets the LED to the specified pattern.
   */
  public void setPattern(Pattern pattern) {
    blinkin.setSpeed(pattern.getValue());
  }

  @Override
  public void periodic() {

  }
}
