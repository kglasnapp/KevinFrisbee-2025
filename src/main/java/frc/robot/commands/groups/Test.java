package frc.robot.commands.groups;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.Timeout;

public class Test extends SequentialCommandGroup {
    public Test() {
        addCommands(
                new DisplayLog(" ---------- Test Autonomous Functions"),
                new Timeout(2),
                new DisplayLog("------------- End Test Autonomous Functions after 2 Second Delay"));
    }
}