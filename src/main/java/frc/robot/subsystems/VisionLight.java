package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Robot;

public class VisionLight {
  private PneumaticHub pHub;
  private Solenoid targetingLight;
  boolean lightState = false;

  public VisionLight() {
    if (Robot.config.PneumaticHUB) {
      pHub = Robot.pHub;
      targetingLight = pHub.makeSolenoid(11);
    }
  }

  public void targetLight(boolean state) {
    // State true to turn on targeting light, false to turn it off
    // The hardware is set for low being on and high being off
    lightState = state;
    targetingLight.set(!state);

  }

  public void toggleTargetingLight() {
    targetLight(!lightState);
  }
}
