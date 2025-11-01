package frc.robot.subsystems;

import static frc.robot.utilities.Util.clip;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.Config.RobotType;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {

    private MotorSRX rightMotor;
    private MotorSRX leftMotor;

    private double rightSpeed = 0;
    private double leftSpeed = 0;
    private boolean brakeMode = true;
    private Double targetAngle = null;
    private double rightJoy;
    private double leftJoy;
    private double yaw;
    private boolean showDriveLog = false;
    private double rightStart = 0;
    private double leftStart = 0;
    private PID positionPID;

    // Parameters for drive train
    private double rampTime = 1; // Change this value in setAggresiveMode() or setMildMode()
    private double slope = 0.0; // Slope of the ramp
    private double intercept = .0; // Offset of the ramp
    private double deadZone = 0.04;
    private boolean turboMode = false;
    private double sensitivity = .8;

    public DifferentialDriveOdometry odometry;
    Pose2d pose;
    Pose2d lastPose;

    public Drivetrain() {
        rightMotor = new MotorSRX("Right", Robot.config.driveRight, Robot.config.driveRightFollow, true);
        leftMotor = new MotorSRX("Left", Robot.config.driveLeft, Robot.config.driveLeftFollow, true);
        rightMotor.setBrakeMode(brakeMode);
        leftMotor.setBrakeMode(brakeMode);
        rightMotor.zeroEncoder();
        leftMotor.zeroEncoder();
        if (Robot.config.invertDrivetrain) {
            logf("Set Right and Left motors to  inverted\n");
            rightMotor.setInverted(false);
            leftMotor.setInverted(true);
        } else {
            logf("Set Right and Left motors to not inverted\n");
            rightMotor.setInverted(true);
            leftMotor.setInverted(false);
        }
        leftMotor.setSensorPhase(true);
        rightMotor.setSensorPhase(false);
        if (Robot.config.getRobotType() == RobotType.Competition) {
            // PID for competion robot
            positionPID = new PID("Drive", .4, 0, .02, 0, 0, -1, 1, false);
        } else {
            // PID for Mini Fast test robot
            positionPID = new PID("Drive", 15, 0, 0, 0, 0, -1, 1, false);
        }
        rightMotor.setPositionPID(positionPID, FeedbackDevice.CTRE_MagEncoder_Relative); // set pid for SRX
        leftMotor.setPositionPID(positionPID, FeedbackDevice.CTRE_MagEncoder_Relative); // set pid for SRX
        rightMotor.setRampOpenLoop(rampTime);
        leftMotor.setRampOpenLoop(rampTime);
    }

    public void setAggresiveMode() {
        logf("***** Set Aggreesive Mode\n");
        rampTime = .5;
        slope = 0;
        intercept = 0;
        deadZone = .04;
        setRampRate(rampTime);
    }

    public void setMildMode() {
        // Keanu liked a=.8, b=0.05, ramp=1.5
        logf("***** Set Mild Mode\n");
        rampTime = 1.5;
        slope = .8;
        intercept = 0.05;
        deadZone = .04;
        setRampRate(rampTime);
    }

    public void setRampRate(double rampTime) {
        rightMotor.setRampOpenLoop(rampTime);
        leftMotor.setRampOpenLoop(rampTime);
    }

    public void setSpeed(double right, double left) {
        setRightMotor(right * 0.5);
        setLeftMotor(left * 0.5);
    }

    public void setLeftMotor(double speed) {
        if (Math.abs(speed) < .005) {
            speed = 0;
        }
        leftMotor.setSpeed(speed);
        leftSpeed = speed;
    }

    public void forcePercentMode() {
        rightMotor.forcePercentMode();
        leftMotor.forcePercentMode();
    }

    public void setRightMotor(double speed) {
        if (Math.abs(speed) < .005)
            speed = 0;
        rightMotor.setSpeed(speed);
        rightSpeed = speed;
    }

    public double getBatteryVoltage() {
        return rightMotor.getBatteryVoltage();
    }

    public double getLeftMotorSpeed() {
        return leftSpeed;
    }

    public double getRightMotorSpeed() {
        return rightSpeed;
    }

    @Override
    public void periodic() {
        yaw = Robot.yaw;
        Rotation2d gyroAngle = Rotation2d.fromDegrees(-yaw + 90);
        if (Robot.count == 10) {
            odometry = new DifferentialDriveOdometry(gyroAngle, rightDistInches(), leftDistInches(),
                    new Pose2d(Robot.config.initialPoseX, Robot.config.initialPoseY, gyroAngle));
        }
        if (Robot.count % 50 == 25) {
            SmartDashboard.putNumber("Right Curremt", round2(rightMotor.getMotorCurrent()));
            SmartDashboard.putNumber("Left Current", round2(leftMotor.getMotorCurrent()));
            SmartDashboard.putNumber("Right Follow", round2(rightMotor.getFollowCurrent()));
            SmartDashboard.putNumber("Left Follow", round2(leftMotor.getFollowCurrent()));
        }
        if (odometry != null) {
            lastPose = pose;
            pose = odometry.update(gyroAngle, leftDistInches(), rightDistInches());
            if (Robot.count % 50 == 0) {
                SmartDashboard.putNumber("PoseX", round2(pose.getX()));
                SmartDashboard.putNumber("PoseY", round2(pose.getY()));
            }
        }
        turboMode = Robot.oi.turboMode();
        if (Robot.oi.driveStraightPressed()) {
            logf("Drive Straight start yaw:%.2f\n", yaw);
            targetAngle = Robot.yaw;
        }
        if (Robot.oi.driveStraightReleased()) {
            logf("Drive Straight finish goal:%.2f yaw:%.2f\n", targetAngle, yaw);
            targetAngle = null;
        }
        if (Robot.driveArcade) {
            arcadeMode();
            return;
        }
        double sensitivity = 1.0;

        rightJoy = Robot.oi.rightJoySpeed();
        leftJoy = Robot.oi.leftJoySpeed();

        if (targetAngle != null) {
            // If Drive straight active make adjustments
            driveStraight();
        }

        rightJoy *= sensitivity;
        leftJoy *= sensitivity;

        // Adjust the ramp rate to make it easier to drive
        // Ramping will also put less strain on the motors
        rightJoy = ramp(rightJoy, rightSpeed, "Right");
        leftJoy = ramp(leftJoy, leftSpeed, "Left");

        showDriveLog = Math.abs(rightJoy) > .06 || Math.abs(leftJoy) > .06;
        if (showDriveLog && Robot.count % 100 == 0) {
            logf("Pos L:%d R:%d Joy L:%.2f R:%.2f\n", leftMotor.getPos(), rightMotor.getPos(), leftJoy,
                    rightJoy);
        }
        setLeftMotor(leftJoy * .6);
        setRightMotor(rightJoy  * .6);
    }

    void driveStraight() {
        double error = Robot.yaw - targetAngle;
        if (error > 10) {
            logf("!!!!! Drive Straight error too positive diff:%.1f yaw:%.1f target:%.3f\n", error,
                    yaw, targetAngle);
            error = 5;
        }
        if (error < -10) {
            logf("!!!!! Drive Straight error too negative diff:%.1f yaw:%.1f target:%.3f\n", error,
                    yaw, targetAngle);
            error = -5;
        }
        // Adjsut speed if too fast
        double averageJoy = (rightJoy + leftJoy) / 1.0;
        // If turbo mode ignore speed limit
        // if (!turboMode) {
        // if (averageJoy > .6)
        // averageJoy = .6;
        // if (averageJoy < -.6)
        // averageJoy = -.6;
        // }
        double factor = error * Math.abs(averageJoy) * 0.035; // Was 0.045
        // Log drive straight data every 2.5 seconds
        if (Robot.count % 12 == 0) {
            logf("Drive Straight targ:%.2f yaw:%.2f err:%.2f avg:%.2f factor:%.2f Fr Dist:%.2f Joy:<%.2f,%.2f>\n",
                    targetAngle, yaw, error,
                    averageJoy, factor, Robot.frontDistance, rightJoy, leftJoy);
        }
        leftJoy = averageJoy - factor;
        rightJoy = averageJoy + factor;
    }

    double rampCubed(double joy, double speed, String side) {
        double result;
        result = 0; // Set to 0 for default case i.e. in dead zone
        if (joy > deadZone) {
            result = intercept + (1 - intercept) * (slope * joy * joy * joy + (1 - slope) * joy);
        }
        if (joy < -deadZone) {
            result = -intercept + (1 - intercept) * (slope * joy * joy * joy + (1 - slope) * joy);
        }
        double act = 0;
        if (side == "Right")
            act = rightMotor.getLastSpeed();
        if (side == "Left")
            act = leftMotor.getLastSpeed();
        if (Math.abs(act) > 0.01) {
            if (Math.abs(lastSpeed - speed) > .05 && side == "Right") {
                logf("Ramp Cubed side:%s joy:%.2f result:%.2f last requested:%.2f motor speed:%.2f slope%.2f\n",
                        side, joy, result, speed, act, slope);
                lastSpeed = speed;
            }
        }
        return result;
    }

    boolean rampCub = true;
    double lastSpeed = 0;

    public double ramp(double joy, double speed, String side) {
        if (rampCub)
            return rampCubed(joy, speed, side);
        double error = Math.abs(speed - joy);
        double newJoy = joy;
        if (error < .04)
            // If small difference simply return joystick value
            return joy;
        if (turboMode) {
            if (joy > speed) {
                newJoy = speed + Robot.config.rampUp;
            } else {
                newJoy = speed - Robot.config.rampDown;
            }
        } else {
            if (joy > speed) {
                newJoy = speed + Robot.config.rampUp;
            } else {
                newJoy = speed - Robot.config.rampDown;
            }
        }
        // Log ramp data if it changed -- only do for right motor to avoid lots of logs
        if (Math.abs(lastSpeed - speed) > .05 && side == "Right") {
            logf("Ramp side:%s in joy:%.2f new joy:%.2f speed:%.2f\n", side, joy, newJoy, speed);
            lastSpeed = speed;
        }
        return newJoy;
    }

    public double rampExp(double joy, double speed, String side) {
        double error = Math.abs(speed - joy);
        if (error < .04)
            return joy;
        boolean joyIsNegative = joy < 0;
        double newJoy = 0.2 * (Math.exp(1.75 * (Math.abs(joy) - 1)));
        newJoy = (joyIsNegative ? -newJoy : newJoy);
        logf("Exp Ramp side:%s in joy:%.2f new joy:%.2f speed:%.2f\n", side, joy, newJoy, speed);
        return newJoy;
    }

    public double getLeftEncoder() {
        return leftMotor.getPos();
    }

    public double getRightEncoder() {
        return rightMotor.getPos();
    }

    // Set the initial position of the robot wheels used in later getDistance Inches
    public void setInitialPosition() {
        rightStart = getRightEncoder();
        leftStart = getLeftEncoder();
    }

    // Get the distance the robot has traveled in inches
    public double getDistanceInches() {
        return (rightDistInches() + rightDistInches()) / 2.0;
    }

    public double rightDistInches() {
        return (getRightEncoder() - rightStart) / Robot.config.driveTicksPerInch;
    }

    public double leftDistInches() {
        return (getLeftEncoder() - leftStart) / Robot.config.driveTicksPerInch;
    }

    public void setBrakeMode(boolean brakeMode) {
        rightMotor.setBrakeMode(brakeMode);
        leftMotor.setBrakeMode(brakeMode);
    }

    public void setDefaultBrakeMode() {
        setBrakeMode(Robot.config.defaultBrakeMode);
    }

    public void resetEncoders() {
        rightMotor.zeroEncoder();
        leftMotor.zeroEncoder();
        rightStart = 0;
        leftStart = 0;
    }

    public void setPosition(int rightTicks, int leftTicks) {
        rightMotor.setPos(rightMotor.getPos() + rightTicks);
        leftMotor.setPos(leftMotor.getPos() + leftTicks);
    }

    private double correctForDeadZone(double speed) {
        if (Math.abs(speed) < deadZone) {
            return 0;
        }
        return speed;
    }

    private void arcadeMode() {
        double yValue = Joysticks.operator.getRawAxis(1) * -1;
        double xValue = Joysticks.operator.getRawAxis(4) * -1;

        yValue = correctForDeadZone(yValue);
        xValue = correctForDeadZone(xValue);

        yValue *= sensitivity;
        xValue *= sensitivity;

        double leftPower = yValue - xValue;
        double rightPower = yValue + xValue;

        leftMotor.setSpeed(clip(leftPower, -1.0, 1.0));
        rightMotor.setSpeed(clip(rightPower, -1.0, 1.0));
    }
}
