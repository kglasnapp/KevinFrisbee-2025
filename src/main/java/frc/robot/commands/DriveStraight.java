package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.Config.RobotType;
import frc.robot.utilities.Util;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Timeout;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveStraight extends Command {

    public enum DriveMode {
        RELATIVE_INCHES, INCHES_FROM_BACK_SENSOR, INCHES_FROM_FRONT_SENSOR, UNTIL_OBSTACLE, INCHES_VIA_LIDAR,
        DRIVE_TO_BALL
    }

    private double distance;
    private double origDistance;
    private DriveMode mode;

    private double speed;
    private double lastSpeed;
    private Drivetrain drive;
    private double timeOut;

    private double desiredYaw;
    private double actualDistance;
    private Double targetAngle = null;
    private Timeout timer = new Timeout();

    public DriveStraight(DriveMode mode, double distance, double speed, double timeOut) {
        addRequirements(Robot.drivetrain);
        this.distance = distance;
        this.origDistance = distance;
        this.mode = mode;
        this.speed = speed;
        this.timeOut = timeOut;
        drive = Robot.drivetrain;
        this.targetAngle = null;
        Util.loginfo("Drive Straight cmd dist:%.1f speed:%.1f\n", distance, speed);
    }

    public DriveStraight(DriveMode mode, double distance, double speed, double timeOut, boolean require) {
        addRequirements(Robot.drivetrain);
        this.distance = distance;
        this.origDistance = distance;
        this.mode = mode;
        this.speed = speed;
        this.timeOut = timeOut;
        drive = Robot.drivetrain;
        this.targetAngle = null;
        Util.loginfo("Drive Straight cmd dist:%.1f speed:%.1f require:%b\n", distance, speed, require);
    }

    public DriveStraight(DriveMode mode, boolean nonstop, double angle, double distance, double speed, double timeOut) {
        addRequirements(Robot.drivetrain);
        this.distance = distance;
        this.origDistance = distance;
        this.mode = mode;
        this.speed = speed;
        this.timeOut = timeOut;
        drive = Robot.drivetrain;
        this.targetAngle = angle;
        Util.loginfo("Drive Straight cmd target angle:%.2f dist:%.1f speed:%.1f\n", angle, distance, speed);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        drive = Robot.drivetrain;
        timer.setTimeout(timeOut);
        desiredYaw = Robot.yaw;
        lastSpeed = 0;
        if (targetAngle != null) {
            desiredYaw = targetAngle;
        }
        drive.setBrakeMode(true);
        drive.setInitialPosition();

        if (mode == DriveMode.DRIVE_TO_BALL) {
            if (Robot.vision.getV()) {
                desiredYaw = Robot.vision.getX() + Robot.yaw;
                Util.logf("Drive Straigt to Ball yaw:%.2f tx:%.2f\n", Robot.yaw, Robot.vision.getX());
                distance = origDistance;
            } else {
                Util.logf("!!!!!!!! Drive Straight to Ball with no vision target\n");
                distance = 2.0; // Drive for 2 inches to stop drive straight
                Robot.vision.takeSnapshot();
            }
        }

        if (mode == DriveMode.INCHES_FROM_FRONT_SENSOR) {
            distance = Robot.frontDistance - origDistance;
        }
        if (mode == DriveMode.INCHES_FROM_BACK_SENSOR) {
            distance = origDistance - Robot.rearDistance;
        }
        double lidarDist = 0;
        if (mode == DriveMode.INCHES_VIA_LIDAR) {
            if (Robot.config.lidar) {
                lidarDist = Robot.lidar.getDistanceInches();
            }
            distance = lidarDist - origDistance;
        }
        speed = Math.copySign(speed, distance);
        Util.logf(
                "++++ Drive Straight cmd mode:%s req dist:%.1f dist:%.1f Request Yaw:%.1f Robot Yaw:%.1f Front:%.1f Rear:%.1f Lidar:%.1f Speed:%.2f\n",
                mode, origDistance, distance, desiredYaw, Robot.yaw, Robot.frontDistance, Robot.rearDistance, lidarDist,
                speed);
    }

    @Override
    public void execute() {
        // Called repeatedly when this Command is scheduled to run
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // Determine the yaw error
        //double distToWall = 0;
        if (mode == DriveMode.DRIVE_TO_BALL) {
            // Riley idea to stop if too close to front wall
            //double distToWall = Robot.frontDistance;
            // if (distToWall < 7) {
            //     Util.logf("Drive to ball -- too close to Wall dist:%.2f\n", distToWall);
            //     return true;
            // }
            // if(distToWall<19 && Robot.intake.beaterBarOut){
            //     Util.logf("Drive to ball -- bar out too close dist:%.2f\n", distToWall);
            //     return true;
            // }
            if(Robot.count % 30 == 9) {
                Robot.vision.takeSnapshot();
            }

            if (!Robot.oi.ballTrackActive()) {
                Util.logf("Track Ball Button Released\n");
                return true;
            }
            desiredYaw = Robot.vision.getX() + Robot.yaw;
        }
        double yawError = Util.normalizeAngle(Robot.yaw - desiredYaw);
        double yawFactor = (yawError * 0.003 * 2);
        // double lidarDist = 0;
        // if (mode == DriveMode.INCHES_VIA_LIDAR && Robot.lidar.isLidarValid()) {
        // if (Robot.config.lidar) {
        // lidarDist = Robot.lidar.getDistanceInches();
        // }
        // distance = lidarDist - origDistance;
        // speed = Math.copySign(speed, distance);
        // } else {
        // speed = 0;
        // return true;
        // }

        actualDistance = drive.getDistanceInches(); // Get actual distance from encoders
        double newSpeed = getNewSpeed(speed, actualDistance);
        drive.setSpeed(newSpeed + yawFactor, newSpeed - yawFactor);
        lastSpeed = newSpeed;

        // Robot.drive.rightMotor.getRevMotor().logAllMotor();
        // Robot.drive.leftMotor.getRevMotor().logAllMotor();
        // if (Robot.count % 5 == 0) {
        Util.logf(
                "Drive Straight cmd time:%.1f speed:%.2f  yaw:<%.1f,E:%.1f> dist:<%.1f,%.1f> enc:<%.0f,%.0f> fact:%.4f FR dist:%.2f\n",
                timer.timeSinceInitializedSeconds(), newSpeed, Robot.yaw, yawError, distance, actualDistance,
                drive.getLeftEncoder(), drive.getRightEncoder(), yawFactor, Robot.frontDistance);
        // }
        if (timer.isTimedOut()) {
            Util.log("???? DriveStraight TimeOut");
            return true;
        }
        // Stop when distance is at less than 1.5 inches
        if (Math.abs(actualDistance) > (Math.abs(distance) - 1.5)) {
            drive.setSpeed(0.0, 0.0);
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        drive.setSpeed(0.0, 0.0);
        // drive.setBrakeMode(Robot.config.defaultBrakeMode);
        Util.logf("---- Drive Straight cmd End mode:%s Dist<%.1f,%.1f,%.1f> Yaw<%.1f,%.1f> Elapsed:%.2f\n", mode,
                distance, actualDistance, actualDistance - distance, Robot.yaw, Robot.yaw - desiredYaw,
                timer.timeSinceInitializedSeconds());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    // Determine speed based upon acceleration and deceleration
    double acceleration = 0.1; // value is number of seconds to achieve full speed -- was .4
    double deceleration = 6; // Value is the number of inches when to start slowing down -- was 12
    boolean noAcceration = true;

    double getNewSpeed(double speed, double actualDistance) {
        if (Config.robotType == RobotType.Competition) {
            speed = ramp(speed, lastSpeed);
            return speed;
        }
        if (noAcceration) {
            return speed;
        }
        double t = timer.timeSinceInitializedSeconds();
        if (t < acceleration) {
            return speed * t / acceleration;
        }
        double remainingDistance = Math.abs(actualDistance - distance) + 2;
        if (remainingDistance < deceleration) {
            return speed * remainingDistance / deceleration;
        }
        return speed;
    }

    public double ramp(double newSpeed, double lastSpeed) {
        if (lastSpeed == newSpeed)
            return lastSpeed;
        if (newSpeed != lastSpeed) {
            if (newSpeed > lastSpeed) {
                lastSpeed += .05;
            } else {
                lastSpeed -= .02;
            }
            // logf("Ramp %s joy:%.2f speed:%.2f\n", side, joy, speed);
        }
        return lastSpeed;
    }
}