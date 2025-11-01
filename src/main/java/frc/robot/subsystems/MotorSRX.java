package frc.robot.subsystems;

import static frc.robot.Robot.count;
import static frc.robot.Robot.logging;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// setSensorPhase() should change the direction reported in the getSelectedSensor*() methods 
// (but not the SensorCollection methods).
// It should also change the direction reported for "PID0" in a self-test snapshot, 
// but not the position reported by "Quad/MagEnc(rel)" 

public class MotorSRX extends SubsystemBase implements MotorDef { 
    private TalonSRX motor;
    private TalonSRX followMotor;
    private String name;
    private int id;
    private int followId;
    private double lastSpeed = 0;
    private int lastPos = 0;
    private boolean myLogging = false;
    private ErrorCode errorCode;
    private boolean sensorPhase = false;
    //private boolean motorInvert = false;
    private FeedbackDevice feedBackDevice = FeedbackDevice.QuadEncoder;

    public MotorSRX(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.id = id;
        this.followId = followId;
        myLogging = logging;
        motor = new TalonSRX(this.id);
        errorCode = motor.configFactoryDefault();
        if (errorCode != ErrorCode.OK) {
            logf("????????? Motor %s Error: %s ??????????\n", name, errorCode);
        }
        if (followId > 0) {
            followMotor = new TalonSRX(followId);
            errorCode = followMotor.configFactoryDefault();
            followMotor.follow(motor);
            if (errorCode != ErrorCode.OK) {
                logf("????????? Follow Motor %s Error: %s ??????????\n", name, errorCode);
                followId = -followId;
            }
        }

        motor.getSensorCollection().setQuadraturePosition(0, 0);
        if (followId > 0)
            logf("Created %s motor ids:<%d,%d> firmware:<%d,%d> voltage:<%.1f,%.1f>\n", name, id, followId,
                    motor.getFirmwareVersion(), followMotor.getFirmwareVersion(), motor.getBusVoltage(),
                    followMotor.getBusVoltage());
        else
            logf("Created %s motor id:%d firmware:%d voltage:%.1f\n", name, id, motor.getFirmwareVersion(),
                    motor.getBusVoltage());

    }

    public int getPos() {
        return (int) motor.getSelectedSensorPosition();
    }

    public void enableLimitSwitch(boolean forward, boolean reverse) {
        if (forward)
            motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        if (reverse)
            motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    public boolean getForwardLimitSwitch()  {
        return motor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimitSwitch() {
        return motor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public  void setBrakeMode(boolean mode) {
        motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
        if (followId > 0)
            followMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setPos(double position) {
        // logf("!!!! SetPos %s to %.1f\n", name, position);
        motor.set(ControlMode.Position, position);
    }

    public void setVelocity(double velocity) {
        // logf("!!!! Set Velocity for %s to %.0f\n", name, velocity);
        motor.set(ControlMode.Velocity, velocity);
    }

    public void setInverted(boolean invert) {
        motor.setInverted(invert);
        if (followId > 0) {
            followMotor.setInverted(invert);
        }
    }

    public double getLastSpeed() {
        return lastSpeed;
    }

    public double getActualVelocity() {
        return motor.getSelectedSensorVelocity(0);
    }

    public void periodic() {
        if (!Robot.config.showMotorData)
            return;
        if (count % 50 == 0 && logging) {
            logPeriodic();
        }
        if (count % 500 == 0)
            updateSmart();
    }

    public void logPeriodic() {
        int pos = motor.getSensorCollection().getQuadraturePosition();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(), pos);
                    logf("%s follow sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d\n", name,
                            followMotor.getMotorOutputPercent(), followMotor.getStatorCurrent(),
                            followMotor.getTemperature(), followMotor.getSelectedSensorVelocity(),
                            followMotor.getSensorCollection().getQuadraturePosition());
                } else {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d\n", name, motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(),
                            motor.getSensorCollection().getQuadraturePosition());
                }
            }
        }
    }

    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
        /*
         * Peak Current and Duration must be exceeded before current limit is activated.
         * When activated, current will be limited to Continuous Current. Set Peak
         * Current params to 0 if desired behavior is to immediately current-limit.
         */
        motor.configPeakCurrentLimit(peakAmps, Robot.config.kTimeoutMs);
        motor.configPeakCurrentDuration(durationMilliseconds, Robot.config.kTimeoutMs);
        motor.configContinuousCurrentLimit(continousAmps, Robot.config.kTimeoutMs);
        motor.enableCurrentLimit(true); // Honor initial setting
    }

    public void updateSmart() {
        SmartDashboard.putNumber(name + " Pos", (int) motor.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber(name + " Cur", round2(motor.getStatorCurrent()));
    }

    public void setSpeed(double speed) {
        if(speed == 0.0){
            motor.set(ControlMode.PercentOutput, 0);
        }
        if (speed != lastSpeed) {
            motor.set(ControlMode.PercentOutput, speed);
        }
        lastSpeed = speed;
    }

    public void setSpeedAbsolute(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
        lastSpeed = speed;
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0);
        lastSpeed = 0;
    }

    public void zeroEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    public void setEncoderPosition(double position) {
        motor.getSensorCollection().setQuadraturePosition((int) position, Robot.config.kTimeoutMs);
    }

    public void setPositionPID(PID pid, FeedbackDevice feedBack) {
        feedBackDevice = feedBack;
        setPositionPID(0, pid);
        PIDToMotor( pid, 0, Robot.config.kTimeoutMs);
    }

    public void forcePercentMode() {
        motor.set(ControlMode.PercentOutput, 0.001);
    }

    public void setVelocityPID(PID pid) { // tod at some point fix this name
        PIDToMotor(pid, 0, Robot.config.kTimeoutMs);
    }

    public  double getMotorVoltage() {
        return motor.getMotorOutputVoltage();
    }

    public double getBatteryVoltage() {
        return motor.getBusVoltage();

    }

    public void PIDToMotor( PID pid, int slot, int timeout) {
        motor.config_kP(slot, pid.kP, timeout);
        motor.config_kI(slot, pid.kI, timeout);
        motor.config_kD(slot, pid.kD, timeout);
        motor.config_kF(slot, pid.kFF, timeout);
        motor.configPeakOutputForward(pid.kMaxOutput);
        motor.configPeakOutputReverse(pid.kMinOutput);
        motor.config_IntegralZone(slot, (int) pid.kIz, timeout);
        motor.configAllowableClosedloopError(slot, pid.allowableCloseLoopError, timeout);
        motor.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
    }

    public double getError() {
        return motor.getClosedLoopError(0);
    }

    public void logMotorVCS() {
        if (Math.abs(lastSpeed) > .02) {
            logf("%s\n", getMotorVCS());
            if (followId > 0) {
                logf("%s\n", getMotorVCS(followMotor));
            }
        }
    }

    public String getMotorVCS() {
        return getMotorVCS(motor);
    }

    private String getMotorVCS(TalonSRX motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputVoltage = motor.getMotorOutputVoltage();
            double supplyCurrent = motor.getSupplyCurrent();
            double statorCurrent = motor.getStatorCurrent();
            return String.format("%s motor volts<%.2f:%.2f> cur<%.2f:%.2f> power<%.2f:%.2f> sp:%.3f", name, bussVoltage,
                    outputVoltage, supplyCurrent, statorCurrent, bussVoltage * supplyCurrent,
                    outputVoltage * statorCurrent, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent();
    }

    public double getFollowCurrent() {
        if (followMotor == null) {
            return 0;
        }
    
        return followMotor.getStatorCurrent();
    }

    public void setSensorPhase(boolean phase) {
        sensorPhase = phase;
        motor.setSensorPhase(phase);
    }

    public  void setPositionPID( int pidIdx, PID pid) {
        // Config the sensor used for Primary PID and sensor direction
        motor.configSelectedFeedbackSensor(feedBackDevice, pidIdx, Robot.config.kTimeoutMs);

        // Ensure sensor is positive when output is positive
       motor.setSensorPhase(sensorPhase);

        // Set based on what direction you want forward/positive to be.
        // This does not affect sensor phase.
       //motor.setInverted(motorInvert);

        /* Config the peak and nominal outputs, 12V means full */
       motor.configNominalOutputForward(0, Robot.config.kTimeoutMs);
       motor.configNominalOutputReverse(0, Robot.config.kTimeoutMs);
       motor.configPeakOutputForward(pid.kMaxOutput, Robot.config.kTimeoutMs);
       motor.configPeakOutputReverse(pid.kMinOutput, Robot.config.kTimeoutMs);

        // Config the allowable closed-loop error, Closed-Loop output will be neutral
        // within this range. See Table in Section 17.2.1 for native units per rotation.
       motor.configAllowableClosedloopError(0, pidIdx, Robot.config.kTimeoutMs);
    }

    public  void setRampClosedLoop(double rate) {
        // Rate is secondsFromNeutralToFull
        motor.configClosedloopRamp(rate);
    }

    public void setRampOpenLoop(double rate) {
        // Rate is secondsFromNeutralToFull
        motor.configOpenloopRamp(rate);
    }

}