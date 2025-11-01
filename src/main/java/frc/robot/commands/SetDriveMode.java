package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import static frc.robot.utilities.Util.logf;

public class SetDriveMode extends Command {
    /** Creates a new ReplaceMeCommand. */
    public SetDriveMode(Robot.DriveMode mode) {
     
        Robot.driveMode = mode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        logf("Set Drive mode to mode:%s\n", Robot.driveMode);
        // Robot.driveMode = Robot.DriveMode.OPERATOR_MILD;
        Robot.driveArcade = false;
        Robot.driveJoy = false;
        switch (Robot.driveMode) {
            case JOY_AGRESSIVE:
                Robot.drivetrain.setAggresiveMode();
                Robot.driveJoy = true;
                Robot.joysticks.initJoySticksForDriveMode();
                break;
            case JOY_MILD:
                Robot.drivetrain.setMildMode();
                Robot.driveJoy = true;
                Robot.joysticks.initJoySticksForDriveMode();
                break;
            case OPERATOR_AGRESSIVE:
                Robot.drivetrain.setAggresiveMode();
                break;
            case OPERATOR_MILD:
                Robot.drivetrain.setMildMode();
                break;
            case ARCADE_AGRESSIVE:
            Robot.drivetrain.setAggresiveMode();
            Robot.driveArcade = true;
            break;
            case ARCADE_MILD:
            Robot.drivetrain.setMildMode();
            Robot.driveArcade = true;
            case NOT_ASSIGNED:
                logf("!!!!!!!!!!!!  Error should never call command SetDriveModee with NOT_ASSIGNED\n");
                break;
        }
        return true;
    }
}