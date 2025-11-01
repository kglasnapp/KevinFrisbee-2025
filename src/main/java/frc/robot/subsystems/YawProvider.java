/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class YawProvider extends SubsystemBase {
  private final AHRS ahrs;
  AnalogInput an_in_0;
  AnalogInput an_in_1;

  public enum PinType {
    DigitalIO, PWM, AnalogIn, AnalogOut
  };

  public YawProvider() {
    ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI, 50);
    an_in_0 = new AnalogInput(getChannelFromPin(PinType.AnalogIn, 0));
    an_in_1 = new AnalogInput(getChannelFromPin(PinType.AnalogIn, 1));
  }

  /**
   * Returns the total accumulated yaw angle in <i>degrees</i> reported by the
   * sensor based on integration of the returned rate from the Z-axis gyro.
   * <p>
   * NOTE: The angle is continuous (meaning its range is beyond 360 degrees),
   * ensuring that there are no discontinuities.
   * <p>
   * This value can be zeroed by calling zeroYaw();
   *
   * @return The current total accumulated yaw angle (Z axis) of the robot in
   *         degrees.
   */

  public double getContinuousYaw() {
    return ahrs.getAngle();
  }

  public double yaw() {
    double yaw = ahrs.getYaw();
    return yaw;
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  /**
   * @return The current rate of change of the yaw in <i>degrees per second</i>.
   */
  public double getYawRate() {
    return ahrs.getRate();
  }

  /**
   * Sets the current yaw to be the center, or 0, yaw. This is usually (the NavX
   * board could have yaw reset capabilities) done by setting an offset value to
   * the current yaw that will be subtracted from subsequent yaws.
   */

  public void zeroYaw() {
    Util.logf("Zero yaw -- previous yaw:%.3f\n", Robot.yaw);
    ahrs.zeroYaw();
  }

  public double getAccelX() {
    return ahrs.getWorldLinearAccelX();
  }

  public double getAccelY() {
    return ahrs.getWorldLinearAccelY();
  }

  public double getAccelZ() {
    return ahrs.getWorldLinearAccelZ();
  }

  /**
   * @return Temperature in degrees Celsius.
   */
  public float getTemperature() {
    return ahrs.getTempC();
  }

  public void navXAnalogUpdate() {
    /* Process Analog Inputs */
    SmartDashboard.putNumber("AnalogIn0", an_in_0.getAverageVoltage());
    SmartDashboard.putNumber("AnalogIn1", an_in_1.getAverageVoltage());
  }

  public final int MAX_NAVX_MXP_DIGIO_PIN_NUMBER = 9;
  public final int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER = 3;
  public final int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER = 1;
  public final int NUM_ROBORIO_ONBOARD_DIGIO_PINS = 10;
  public final int NUM_ROBORIO_ONBOARD_PWM_PINS = 10;
  public final int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS = 4;

  /* getChannelFromPin( PinType, int ) - converts from a navX MXP */
  /* Pin type and number to the corresponding RoboRIO Channel */
  /* Number, which is used by the WPI Library functions. */

  public int getChannelFromPin(PinType type, int io_pin_number)
      throws IllegalArgumentException {
    int roborio_channel = 0;
    if (io_pin_number < 0) {
      throw new IllegalArgumentException("Error:  navX MXP I/O Pin #");
    }
    switch (type) {
      case DigitalIO:
        if (io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER) {
          throw new IllegalArgumentException("Error:  Invalid navX MXP Digital I/O Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS +
            (io_pin_number > 3 ? 4 : 0);
        break;
      case PWM:
        if (io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER) {
          throw new IllegalArgumentException("Error:  Invalid navX MXP Digital I/O Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
        break;
      case AnalogIn:
        if (io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER) {
          throw new IllegalArgumentException("Error:  Invalid navX MXP Analog Input Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
        break;
      case AnalogOut:
        if (io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER) {
          throw new IllegalArgumentException("Error:  Invalid navX MXP Analog Output Pin #");
        }
        roborio_channel = io_pin_number;
        break;
    }
    return roborio_channel;
  }
}