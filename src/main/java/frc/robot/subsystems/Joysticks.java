package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Robot;

public class Joysticks {
    public static Joystick leftJoy;
    public static Joystick rightJoy;
    public static Joystick operator;
    public static Joystick driver;
    public static boolean joysticksPresent = false;

    public Joysticks() {
        if (Robot.driveJoy) {
            logf("Init Joy Stick Drive\n");
            leftJoy = new Joystick(0);
            rightJoy = new Joystick(1);
        }
        if (Robot.config.operatorPadEnabled) {
            operator = new Joystick(2);
        }
        if (Robot.config.driverPadEnabled) {
            driver = new Joystick(3);
        }
        String rightName = DriverStation.getJoystickName(0);
        String leftName = DriverStation.getJoystickName(1);
        if (rightName == "" || leftName == "")
            joysticksPresent = false;
        else
            joysticksPresent = true;
        rightName = (rightName == "" ? "Not Present" : rightName);
        leftName = (leftName == "" ? "Not Present" : leftName);
        logf("Joysticks R:%s L:%s operator pad:%s driver pad:%s operator:%s\n", rightName, leftName,
                DriverStation.getJoystickName(2), DriverStation.getJoystickName(3), DriverStation.getJoystickName(4));
    }

    public boolean getJoysPresent() {
        return joysticksPresent;
    }

    public void initJoySticksForDriveMode() {
        logf("Set Joy Stick Drive Mode\n");
        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);
    }

    public Joystick getRightJoy() {
        return rightJoy;
    }

    public Joystick getLeftJoy() {
        return leftJoy;
    }

    public Joystick getDriverPad() {
        return driver;
    }

    public Joystick getOperatorPad() {
        return operator;
    }

    public Joystick getOperatorJoy() {
        return operator;
    }

    public void setLeftRumble(double value) {
        operator.setRumble(RumbleType.kLeftRumble, value);
    }

    public void setRightRumble(double value) {
        operator.setRumble(RumbleType.kRightRumble, value);
    }
}