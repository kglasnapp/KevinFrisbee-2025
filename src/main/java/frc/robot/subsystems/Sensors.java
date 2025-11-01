/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Config.RobotType;

public class Sensors extends SubsystemBase {

  private AnalogInput rearBall;
  private AnalogInput frontBall;
  private AnalogInput frisbeeIndicator;
  private AnalogInput frontDistance;
  private AnalogInput rearDistance;
  private double lastDist;
  int lastPulse = 0;
  Solenoid light;

  public Sensors() {
    if (Robot.config.getRobotType() == RobotType.Kevin) {
      frisbeeIndicator = new AnalogInput(0);
    } else {
      frontBall = new AnalogInput(1);
      rearBall = new AnalogInput(0);
      frontDistance = new AnalogInput(2);
      rearDistance = new AnalogInput(3);
    }
    if (Robot.config.PneumaticHUB)
      light = Robot.pHub.makeSolenoid(Robot.config.blinkerChannel);
  }

  @Override
  public void periodic() {
    if (Robot.config.ultraSonicDistance) {
      Robot.frontBallVoltage = frontBall.getVoltage();
      Robot.rearBallVoltage = rearBall.getVoltage();

      if (Robot.count % 15 == 7) {
        updateSmartDashboard();
      }
    }
    double distInches = Robot.frontDistance;
    blinkForLight(80, distInches); // Set sweat spot to 80 inches -- for the long shoot
    if (Robot.count % 250 == 5 && frontBallPresent() && rearBallPresent()) {
      logf("Front ball:%.2f Rear Ball:%.2f\n", Robot.frontBallVoltage, Robot.rearBallVoltage);
    }
  }

  public boolean rearBallPresent() {
    return Robot.rearBallVoltage > 2.0;
  }

  public boolean frontBallPresent() {
    return Robot.frontBallVoltage > 2.0;
  }

  // If voltage below .25 the sensor is not valid
  public boolean rearBallValid() {
    if (rearBall == null) {
      return false;
    }
    return rearBall.getVoltage() > .25;
  }

  public boolean frisbeeValid() {
    if (frisbeeIndicator == null) {
      return false;
    }
    return frisbeeIndicator.getVoltage() > 2.0;
  }

  // If voltage below .25 the sensor is not valid
  public boolean frontBallValid() {
    if (frontBall == null) {
      return false;
    }
    return frontBall.getVoltage() > .25;
  }

  public double getFrontDistance() {
    if (frontDistance == null) {
      return 0.0;
    }
    double volts = frontDistance.getVoltage();
    if (Robot.config.ultra1030Front) {
      if (volts < .060) {
        return 0;
      } else {
        return round2(volts / .0098);
      }
    }
    if (volts < .2)
      return 0;
    return round2(volts * 42.3);
  }

  public double getRearDistance() {
    if (rearDistance == null) {
      return 0.0;
    }
    double volts = rearDistance.getVoltage();
    if (Robot.config.ultra1030Rear) {
      if (volts < .060) {
        return 0;
      } else {
        return round2(volts / .098);
      }
    }
    if (volts < .2)
      return 0;
    return round2(volts * 44);
  }

  void updateSmartDashboard() {
    if (Robot.config.getRobotType() == RobotType.Kevin) {
      SmartDashboard.putNumber("Frisbee Voltage", frisbeeIndicator.getVoltage());
      SmartDashboard.putBoolean("Frisbee Valid", frisbeeValid());
    }

    if (Robot.config.getRobotType() == RobotType.Sibling) {
      SmartDashboard.putNumber("Bottom Ball", frontBallPresent() ? 1 : 0);
      SmartDashboard.putNumber("Top Ball", rearBallPresent() ? 1 : 0);
    }
    if (Robot.config.getRobotType() == RobotType.Competition) {
      SmartDashboard.putNumber("Front Ball", frontBallPresent() ? 1 : 0);
      SmartDashboard.putNumber("Rear Ball", rearBallPresent() ? 1 : 0);
    }
    SmartDashboard.putNumber("Front Dist", Robot.frontDistance);
    SmartDashboard.putNumber("Rear Dist", Robot.rearDistance);
  }

  public void blinkForLight(double target, double distInches) { // target value: 20, 15, 5
    // light is solid when near target (half foot), blinks fast when when slightly
    // farther (foot), blinks slow when farther (foot and a half)
    if (light == null)
      return;
    int pulse = -1;
    if (target < 0.1) {
      light.set(false);
      return;
    }
    if (distInches <= (target + 5) && distInches >= (target - 5)) { // smallest distance
      pulse = 0;
    } else if (distInches <= (target + 16) && distInches >= (target - 16)) {
      pulse = 8; // Pulse every 160 milliseconds
    } else if (distInches <= (target + 26) && distInches >= (target - 26)) {
      pulse = 4; // Pulse every 80 milliseconds
    }
    if (pulse == 0)
      light.set(true);
    else if (Robot.count % pulse == 0 && pulse > 0)
      light.set(!light.get());
    else if (pulse == -1)
      light.set(false);
    if (distInches != lastDist) {
      if (target > 0 && Math.abs(lastDist - distInches) > 4 && distInches < 80) {
        // logf("************** Dist:%.2f pulse:%d light:%b prep:%.2f\n", distInches,
        // pulse, light.get(), target);
      }
      lastDist = distInches;
      lastPulse = pulse;
    }
  }
}