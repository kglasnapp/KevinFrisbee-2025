/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import static frc.robot.utilities.Util.logf;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import static frc.robot.utilities.Util.logf;

public class ZeroYawAndPosition extends Command {

    double initialX = Robot.config.initialPoseX;
    double initialY = Robot.config.initialPoseY;
    Rotation2d gyroAngle;

    public ZeroYawAndPosition() {
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        if (Robot.yawNavX != null) {
            Robot.yawNavX.zeroYaw();
        }
        // Reset the drive encodeers for easy debugging
        Robot.drivetrain.resetEncoders();
        
        if (Robot.drivetrain != null) {
            double yaw = Robot.yaw;
            gyroAngle = Rotation2d.fromDegrees(-yaw + 90);
            Robot.drivetrain.odometry = new DifferentialDriveOdometry(gyroAngle, Robot.drivetrain.rightDistInches(), Robot.drivetrain.leftDistInches(),
                    new Pose2d(initialX, initialY, gyroAngle));
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        logf("Zero Yaw and Position yaw:%.2f x:%.3f y:%.3f gyro:%.2f\n", Robot.yaw, initialX, initialY,  gyroAngle.getDegrees() ) ;
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}
