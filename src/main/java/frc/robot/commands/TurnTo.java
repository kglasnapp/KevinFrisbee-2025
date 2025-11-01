/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.RunningAverage;
import frc.robot.utilities.Util;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.Config.RobotType;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MotorSRX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.Timeout;

/**
 * Command to perform a turnTo function Various modes exist ABSOLUTE, RELATIVE,
 * GRID_CW, GRID_CCW, VISION User specifies the mode and desired angle
 */

public class TurnTo extends Command {

    public enum TurnMode {
        ABSOLUTE, RELATIVE, GRID_CW, GRID_CCW, VISION
    }

    private double targetAngle;
    private double requestedAngle;
    private TurnMode mode;
    private final double gridError = 3.0;
    private double initialDelta = 0;
    private int shortTurnCount = 0;
    private int count = 0;
    private RunningAverage runAvg;

    // Varibles for two PID case
    private posPID pidR;
    private posPID pidL;
    private boolean rOK = false;
    private boolean lOK = false;

    // Variables for one PID
    private PIDController pid;
    private double setPoint;
    // Tunned with kI
    // private double kP = 0.01225; // 0.00972000 * 2.0; // last stable: 0.00910000
    // private double kI = 0.06940; // 0.00004530; // last stable: 0.00003000
    // private double kD = 0.000540; // 0.00083000 * 1.4; // last stable: 0.00085000

    // Tunned for 4 degress
    // private double kP = 0.01633; // 0.00972000 * 2.0; // last stable: 0.00910000
    // private double kI = 0; // 0.00004530; // last stable: 0.00003000
    // private double kD = 0.000720; // 0.00083000 * 1.4; // last stable: 0.00085000
    // Tunned for 45 degrees
    // private double kP = 0.01633; // 0.00972000 * 2.0; // last stable: 0.00910000
    // private double kI = 0; // 0.00004530; // last stable: 0.00003000
    // private double kD = 0.001361; // 0.00083000 * 1.4; // last stable: 0.00085000
    // ZN math
    // private double kP = 0.00674; // 0.00972000 * 2.0; // last stable: 0.00910000
    // private double kI = 0.03817; // 0.00004530; // last stable: 0.00003000
    // private double kD = 0.000792; // 0.00083000 * 1.4; // last stable: 0.00085000

    // Best from Keith / Anothy

    private double kP = 0.00972000; // last stable: 0.00910000
    private double kI = 0.00004530; // last stable: 0.00003000
    private double kD = 0.00083000 * 1.4; // last stable: 0.000850

    private double kMaxOutput = .25 * 2;
    private double kMinOutput = -.25 * 2;
    private double origAdjust;

    private Timeout timer = new Timeout();

    private double ticksPerDegree = Robot.config.wheelBase * Math.PI * Robot.config.driveTicksPerInch * (45.0 / 39.0)
            / 360;

    private enum Mode {
        ONE_PID_YAW, TWO_PID, SPIN_UNTIL, ONE_PID_TICKS, POSITION_PID
    }

    private Mode type = Mode.ONE_PID_YAW;
    private boolean fast = false;

    private Drivetrain drive;

    private enum direction {
        RIGHT, LEFT
    };

    private direction actualDir;

    public TurnTo(TurnMode mode, double angle) { // positive angle CW, negative angle CCW
        addRequirements(Robot.drivetrain);
        this.requestedAngle = angle;
        this.mode = mode;
        this.type = Mode.SPIN_UNTIL;
        fast = false;
        Util.loginfo("Create TurnTo %.1f %s\n", angle, mode);
    }

    public TurnTo(TurnMode mode) {
        this.requestedAngle = 90;
        this.mode = mode;
        Util.loginfo("Create TurnTo no angle  %s\n", mode);
        fast = false;
        addRequirements(Robot.drivetrain);
    }

    public TurnTo(TurnMode mode, double angle, boolean fast) { // positive angle CW, negative angle CCW
        addRequirements(Robot.drivetrain);
        this.requestedAngle = angle;
        this.mode = mode;
        this.fast = fast;
        type = Mode.POSITION_PID;
        Util.loginfo("Create pos pid based TurnTo %.1f %s fast:%b\n", angle, mode, this.fast);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        drive = Robot.drivetrain;
        count = 0;
        runAvg = new RunningAverage(5);
        // Defaults to turn Absolute Angle
        targetAngle = requestedAngle;

        if (mode == TurnMode.RELATIVE)
            targetAngle = Robot.yaw + targetAngle;

        if (mode == TurnMode.GRID_CCW)
            targetAngle = Math.floor((Robot.yaw - gridError) / targetAngle) * targetAngle;

        // Math.floor((currentYaw + gridError) / 90) * 90 + 90
        if (mode == TurnMode.GRID_CW)
            targetAngle = Math.floor((Robot.yaw + gridError) / targetAngle) * targetAngle + targetAngle;

        targetAngle = Util.normalizeAngle(targetAngle);
        Util.logf("++++ TurnTo %s mode: %s req angle:%.1f tar angle:%.1f  yaw:%.1f ticks\\degrees:%.0f\n", type, mode,
                requestedAngle, targetAngle, Robot.yaw, ticksPerDegree);
        timer.setTimeout(10.0);
        drive.setBrakeMode(true);

        initialDelta = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
        if (Math.abs(initialDelta) < 9) {
            shortTurnCount = 2;
        }
        // double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) -
        // Util.normalizeAngle(Robot.yaw));
        switch (type) {
            case ONE_PID_YAW:
                pid = new PIDController(kP, kI, kD);
                pid.setTolerance(.1);
                Util.logf("posPID with  p:%.8f i:%.8f d:%.8f setPoint:%.1f delta:%.1f\n", kP, kI, kD,
                        setPoint, initialDelta);
                break;
            case TWO_PID:
                // pidL = new posPID("Left", Robot.drive.leftMotor);
                // pidR = new posPID("Right", Robot.drive.rightMotor);
                break;
            case SPIN_UNTIL:
                break;
            case ONE_PID_TICKS:
                // PID parms for a position PID
                kP = 0.00025 * 1.3 * .05;
                kI = 0; // 1e-4;
                kD = 0.000005;
                kMaxOutput = .25;
                kMinOutput = -.25;
                pid = new PIDController(kP, kI, kD);
                pid.setTolerance(50);
                Util.logf("one pid ticks with p:%.8f i:%.8f d:%.8f setPoint:%.1f delta:%.1f \n", kP,
                        kI, kD, setPoint, initialDelta);
                break;
            case POSITION_PID:
                int ticks = (int) (Robot.config.driveTicksPerInch * initialDelta * .3);
                drive.setPosition(-ticks, ticks);
                actualDir = (initialDelta < 0) ? direction.LEFT : direction.RIGHT;
                // Util.logf("Set turn pos pid delta:%.2f yaw:%.2f ticks:%d direction:%s\n", initialDelta, Robot.yaw,
                //         ticks, actualDir);
                break;
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        switch (type) {
            case ONE_PID_YAW:
                return isFinishedOnePID();
            case TWO_PID:
                return isFinishedTwoPIDMode();
            case SPIN_UNTIL:
                return isFinishedSpinMode();
            case ONE_PID_TICKS:
                return isFinishOnePIDTicks();
            case POSITION_PID:
                count++;
                if (count > 50) {
                    return true;
                }
                double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
                if (actualDir == direction.LEFT && error > 0) {
                    Util.logf("Got Left yaw:%.2f error:%.2f\n", Robot.yaw, error);
                    return true;
                }
                if (actualDir == direction.RIGHT && error < 0) {
                    Util.logf("Got Right yaw:%.2f error:%.2f\n", Robot.yaw, error);
                    return true;
                }
                Util.logf("Fast Turn yaw:%.2f error:%.2f enc:<L:%.2f,R:%.2f>\n", Robot.yaw, error,
                        Robot.drivetrain.getRightEncoder(),
                        Robot.drivetrain.getLeftEncoder());
                return false;

        }
        return isFinishedTwoPIDMode();
    }
    // Make this return true when this Command no longer needs to run execute()

    protected boolean isFinishedSpinMode() {
        count++;
        if (timer.isTimedOut()) {
            Util.logf("??? TurnTo timed out requested:%.1f yaw:%.1f error:%.1f\n", targetAngle, Robot.yaw,
                    Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw)));
            return true;
        }
        double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
        Util.logf("TurnTo time:%f yaw:%.1f target:%.2f error:%.2f enc<%f,%f> error:%.1f count:%d\n",
                timer.timeSinceInitializedSeconds(), Robot.yaw, targetAngle, error, drive.getRightEncoder(),
                drive.getLeftEncoder(), error, count);
        double speed = 0.23;
        if(Math.abs(error) < 2.5) {
            return true;
        }
        if (error > 8) {
            drive.setSpeed(-speed, speed);
            count = 0;
            return false;
        }
        if (error < -8) {
            drive.setSpeed(speed, -speed);
            count = 0;
            return false;
        }
        speed = .16;
        if (count < 6) {
            if (error > 0)
                drive.setSpeed(speed, -speed);
            else
                drive.setSpeed(-speed, speed);
            return false;
        }
        return true;
    }

    boolean isFinishedTwoPIDMode() {
        count++;
        if (timer.isTimedOut()) {
            Util.logf("??? PID TurnTo timed out requested:%.1f yaw:%.1f error:%.1f\n", targetAngle, Robot.yaw,
                    Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw)));
            return true;
        }
        // Use the difference between initial yaw and yaw to determine a correction
        double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
        int ticks = (int) (error * ticksPerDegree);
        runAvg.add(error);
        // Set to - for Rev2 Min
        rOK = pidR.doPID(ticks);
        lOK = pidL.doPID(ticks);
        if (count % 2 == 0) {
            Util.logf("Two %.3f yaw:%.1f er:%.1f avg:%.1f r:%s l:%s\n", timer.timeSinceInitializedSeconds(), Robot.yaw,
                    error,
                    runAvg.getAverage(), pidR.getData(), pidL.getData());
        }
        if (rOK && lOK) {
            Util.logf("*** TurnTo angle achieved, error:%.1f target:%.1f yaw:%.1f\n", error, targetAngle, Robot.yaw);
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    // Called once after isFinished returns true
    public void end(boolean interrupted) {
        if (type == Mode.TWO_PID) {
            pidL.close();
            pidR.close();
        } else {
            drive.forcePercentMode();
            drive.setSpeed(0.0, 0.0);
        }
        drive.setDefaultBrakeMode();
        double err = Math.abs(targetAngle - Robot.yaw);
        Util.logf("---- TurnTo %s end time:%.4f target angle:%.2f yaw:%.2f\n", type, timer.timeSinceInitializedMilli(),
                targetAngle, Robot.yaw, err);
        SmartDashboard.putNumber("Turn Error", Util.round2(err));
        SmartDashboard.putNumber("Turn Time", Util.round4(timer.timeSinceInitializedMilli()));
    }

    boolean shortTurn(double angle) {
        shortTurnCount--;
        double speed = 0.3;
        if (angle < 0)
            speed = -speed;
        double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
        Util.logf("Short turn error:%.1f count:%d speed:%.1f\n", error, shortTurnCount, speed);
        if (shortTurnCount < 0)
            return true;
        drive.setSpeed(-speed, speed);
        return false;
    }

    public class posPID {
        int spCount = 0;
        PIDController pid;
        MotorSRX motor;
        double setP = 0;
        double kP = 0.00025 * 1.3 * .05;
        double kI = 0; // 1e-4;
        double kD = 0.000005;
        // double kD = 0;
        double kMaxOutput = .25;
        double kMinOutput = -.25;
        String name = "";
        double origAdjust;
        double startEnc = 0;
        boolean atSp = false;
        boolean ok = false;

        public posPID(String name, MotorSRX motor) {
            this.motor = motor;
            this.name = name;
            pid = new PIDController(kP, kI, kD);
            pid.setTolerance(5000);
            pid.setSetpoint(setPoint);
            startEnc = -motor.getPos();
            Util.logf("posPID %s with  p:%.8f i:%.8f d:%.8f enc:%.0f\n", name, kP, kI, kD, startEnc);
        }

        String getData() {
            return String.format("%b er:%.0f oad:%.3f p:%.7f", atSp, pid.getPositionError(), origAdjust, pid.getP());
        }

        void setNewSetpoint(double ticks) {
            pid.setSetpoint(ticks);
        }

        void adjustP(double adjust) {
            pid.setP(kP * adjust);
        }

        double getError() {
            return pid.getPositionError();
        }

        double getOrigAdjust() {
            return origAdjust;
        }

        boolean doPID(double ticks) {
            if (setP == 0)
                setP = ticks + startEnc;
            if (ok)
                return true;
            double enc = -motor.getPos();
            double adjust = pid.calculate(enc, setP);
            origAdjust = adjust;
            if (adjust > kMaxOutput) {
                adjust = kMaxOutput;
            }
            if (adjust < kMinOutput) {
                adjust = kMinOutput;
            }
            double error = Math.abs(pid.getPositionError()) / ticksPerDegree;
            if (Math.abs(error) < .5) {
                adjustP(5);
            } else if (Math.abs(error) < 1) {
                adjustP(4);
            } else if (Math.abs(error) < 3) {
                adjustP(3);
            } else if (Math.abs(error) < 4) {
                adjustP(2);
            } else {
                adjustP(1);
            }

            motor.setSpeed(adjust);
            // double error = setP - enc;
            atSp = pid.atSetpoint();
            if (atSp)
                spCount++;
            if ((atSp && spCount > 10)) {
                ok = true;
                return true;
            }
            ok = false;
            return false;
        }

        void close() {
            pid.reset();
            pid.close();
            motor.setSpeed(0.0001);
        }

    }

    boolean isFinishedOnePID() {
        double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
        if (timer.isTimedOut()) {
            Util.logf("??? TurnTo one PID timed out requested:%.1f yaw:%.1f error:%.1f\n", setPoint, Robot.yaw, error);
            return true;
        }
        runAvg.add(error);
        if (Config.robotType == RobotType.MiniSRX) {
            if (Math.abs(error) < .5) {
                pid.setP(kP * 11);
            } else if (Math.abs(error) < 1) {
                pid.setP(kP * 9);
            } else if (Math.abs(error) < 2) {
                pid.setP(kP * 7);
            } else if (Math.abs(error) < 3) {
                pid.setP(kP * 5);
            } else if (Math.abs(error) < 5) {
                pid.setP(kP * 4);
            }
            if (Math.abs(error) < 6 && runAvg.deviation() < .01) {
                pid.setP(kP * 2);
            }
        }
        if (Config.robotType == RobotType.Competition) {
            if (Math.abs(error) < .5) {
                pid.setP(kP * 3);
            } else if (Math.abs(error) < 1) {
                pid.setP(kP * 3);
            } else if (Math.abs(error) < 2) {
                pid.setP(kP * 2);
            } else if (Math.abs(error) < 3) {
                pid.setP(kP * 1.5);
            } else
                pid.setP(kP);

        }
        // if (Math.abs(error) < 6 && runAvg.deviation() < .01) {
        // pid.setP(kP * 2);
        // }
        double adjust = pid.calculate(-error, 0);
        origAdjust = adjust;
        if (adjust > kMaxOutput) {
            adjust = kMaxOutput;
        }
        if (adjust < kMinOutput) {
            adjust = kMinOutput;
        }
        drive.setSpeed(-adjust, adjust);
        boolean sp = pid.atSetpoint();
        Util.logf(
                "TurnTo one PID time:%.3f yaw:%.1f Error:%.3f sp:%b adjust:%6.3f orig:%6.3f kP:%.5f count:%d avg:%.3f dev:%.3f\n",
                timer.timeSinceInitializedSeconds(), Robot.yaw, error, sp, adjust, origAdjust, pid.getP(), count,
                runAvg.getAverage(), runAvg.deviation());
        if (sp || Math.abs(error) < .3
                || ((Math.abs(runAvg.deviation()) < .2) && (timer.timeSinceInitializedSeconds() > .4)))
            count++;
        if (count > 7) {
            pid.reset();
            pid.close();
            return true;
        }
        return false;
    }

    boolean isFinishOnePIDTicks() {
        double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
        if (timer.isTimedOut()) {
            Util.logf("??? TurnTo one PID timed out requested:%.1f yaw:%.1f error:%.1f\n", setPoint, Robot.yaw, error);
            return true;
        }
        runAvg.add(error);
        if (Config.robotType == RobotType.MiniSRX) {
            if (Math.abs(error) < .5) {
                pid.setP(kP * 11);
            } else if (Math.abs(error) < 1) {
                pid.setP(kP * 9);
            } else if (Math.abs(error) < 2) {
                pid.setP(kP * 7);
            } else if (Math.abs(error) < 3) {
                pid.setP(kP * 5);
            } else if (Math.abs(error) < 5) {
                pid.setP(kP * 4);
            }
            if (Math.abs(error) < 6 && runAvg.deviation() < .01) {
                pid.setP(kP * 2);
            }
        }
        // if (Math.abs(error) < 5)
        // pid.setP(kP * 1.5);
        // else
        // pid.setP(kP);
        double adjust = pid.calculate(-error, 0);
        origAdjust = adjust;
        if (adjust > kMaxOutput) {
            adjust = kMaxOutput;
        }
        if (adjust < kMinOutput) {
            adjust = kMinOutput;
        }
        int ticks = (int) (error * ticksPerDegree);
        drive.setSpeed(-adjust, adjust);

        boolean sp = pid.atSetpoint();
        Util.logf(
                "TurnTo one PID time:%.3f yaw:%.1f Error:%.3f ticks:%d sp:%b adjust:%6.3f orig:%6.3f kP:%.5f count:%d avg:%.3f dev:%.3f\n",
                timer.timeSinceInitializedSeconds(), Robot.yaw, error, ticks, sp, adjust, origAdjust, pid.getP(), count,
                runAvg.getAverage(), runAvg.deviation());
        if (sp || Math.abs(error) < .3
                || ((Math.abs(runAvg.deviation()) < .2) && (timer.timeSinceInitializedSeconds() > .4)))
            count++;
        if (count > 3) {
            pid.reset();
            pid.close();
            return true;
        }
        return false;
    }
}