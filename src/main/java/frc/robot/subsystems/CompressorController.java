package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

public class CompressorController extends SubsystemBase {
    private Compressor compressor;
    private boolean stopped = false;
    private boolean enabled;

    // Create the constructor for the compressor
    public CompressorController() {
        enabled = Robot.config.enableCompressor;
        Util.logf("Compressor enabled: %b\n", enabled);
        if (enabled) {
            compressor = new Compressor( PneumaticsModuleType.CTREPCM);
            compressor.isEnabled();
        }
    }

       // return true if compressor is operating on closed-loop mode
    public boolean isCompressorEnabled() {
        return compressor.isEnabled();
    }

    // check to see if the compressor is actually running
    public boolean isRunning() {
        return compressor.isEnabled();
    }

    // stop the compressor if it is not stopped
    // return true if stopped
    public boolean stop() {
        if (!Robot.config.enableCompressor)
            return false;
        if (!stopped) {
            compressor.disable();
            stopped = true;
            return true;
        }
        return false;
    }

    // restart the compressor if it was stopped
    // return true if restarted
    public boolean restart() {
        if (stopped && enabled) {
            compressor.enableDigital();
            stopped = false;
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "CompressorController [enabled=" + enabled + ", stopped=" + stopped + "]";
    }
}