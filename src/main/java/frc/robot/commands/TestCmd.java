
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class TestCmd extends Command {

    int testNumber = 0;
    public TestCmd() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        testNumber = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        testNumber += 1;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
       return true;
    }
}
