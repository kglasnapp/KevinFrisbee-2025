package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.*;

/**
 * An example command. You can replace me with your own command.
 */
public class DisplayLog extends Command {
  private String s;

  public DisplayLog(String s) {
    // Use requires() here to declare subsystem dependencies
    this.s = s;
  }

  // Called just before this Command runs the first time

 public void initialize() {
    Util.logf("%s\n", s);
  }

  // Called repeatedly when this Command is scheduled to run

  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true

  public void end() {
  }

}
