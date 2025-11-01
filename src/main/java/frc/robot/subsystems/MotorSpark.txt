package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

import static frc.robot.utilities.Util.round2;
import static frc.robot.Robot.count;

/**
 * SPARK MAX controllers are initialized over CAN by constructing a CANSparkMax
 * object
 *
 * The CAN ID, which can be configured using the SPARK MAX Client, is passed as
 * the first parameter
 *
 * The motor type is passed as the second parameter. Motor type can either be:
 * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
 * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
 *
 */

// Add logic for limit switches and PID
public class MotorSpark extends SubsystemBase implements MotorDef {
    // public class MotorSpark extends SubsystemBase {
    private CANSparkMax motor;
    private CANSparkMax followMotor;
    private String name;
    private int followId;
    private double lastSpeed;
    private long encoder;
    private long lastEncoder;
    private boolean encoderHit = true; // Flag use to indicate if encoder has changed, if do a log
    private double current;
    private double temperature;
    private double velocity;
    private long encoderRear;
    private double currentRear;
    private double temperatureRear;
    private double velocityRear;
    private long periodicHit = 0;
    private SparkMaxLimitSwitch forwardSwitch;
    private SparkMaxLimitSwitch reverseSwitch;
    private boolean myLogging = false;
    private double desiredPositionTicks;
    public double posConversionFactor = 1000;
    public RelativeEncoder relEncoder;
    public RelativeEncoder relEncoderFollow;
    private int lastPos = 0;

    MotorSpark(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.followId = followId;
        myLogging = logging;
        motor = new CANSparkMax(id, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        if (followId > 0) {
            followMotor = new CANSparkMax(followId, MotorType.kBrushless);
            followMotor.restoreFactoryDefaults();
            followMotor.follow(motor);
            followMotor.setIdleMode(IdleMode.kBrake);
            relEncoderFollow = followMotor.getEncoder();
        }
        // Force Brake Mode for all Rev Motors
        motor.setIdleMode(IdleMode.kBrake);
        relEncoder = motor.getEncoder();
        relEncoder.setPosition(0.0);
        relEncoder.setPositionConversionFactor(posConversionFactor);

        forwardSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        if (followId > 0)
            logf("Created %s dual motors ids:<%d,%d> firmware:<%s,%s> Position Conversion Factor:<%f,%f>\n", name, id,
                    followId, motor.getFirmwareString(), followMotor.getFirmwareString(),
                    relEncoder.getPositionConversionFactor(), relEncoderFollow.getPositionConversionFactor());
        else
            logf("Created %s motor id:%d firmware:%s Position Conversion Factor:%f\n", name, id,
                    motor.getFirmwareString(), relEncoder.getPositionConversionFactor());
    }

    public void enableLimitSwitch(boolean forward, boolean reverse) {
        forwardSwitch.enableLimitSwitch(forward);
        reverseSwitch.enableLimitSwitch(reverse);
    }

    public void setPositionPID(int slot, PID pid) {
        // Sensor phase for Rev motors is not needed
        // https://www.chiefdelphi.com/t/sparkmax-setinverted-affecting-pid-position-control/343315
        PIDToMotor(pid, 0, 0);
    }

    public void setSensorPhase(boolean value) {
        // Sensor phase for Rev motors is not needed since encoder is part of the motor
    }

    public void setInverted(boolean value) {
        motor.setInverted(value);
    }

    public void setVelocityPID(PID pid) {
        PIDToMotor(pid, 1, 0);
    }

    public void forcePercentMode() {
       // use the function set speed to force percent mode
    }

    public void setCurrentLimit(int stallLimit, int freeLimit, int timeOut) {
        motor.setSmartCurrentLimit(stallLimit, freeLimit);
        if (followMotor != null)
            followMotor.setSmartCurrentLimit(stallLimit, freeLimit);
    }

    public void setBrakeMode(boolean mode) {
        logf("----- Brake Mode for %s %b\n", name, mode);
        motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        if (followMotor != null)
            followMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public double getSpeed() {
        // return lastSpeed;
        return relEncoder.getVelocity();
    }

    public double getActualVelocity() {
        // return lastSpeed;
        return relEncoder.getVelocity();
    }

    public double getRPM() {
        return relEncoder.getVelocity();
    }

    CANSparkMax getMotor() {
        return motor;
    }

    CANSparkMax getFollowMotor() {
        return followMotor;
    }

    public double getError() {
        // Seems that REV does not have anything to get the closed loop error???
        return 0.0;
    }

    public void setSpeed(double speed) {
        // Make up for bug in Rev motor in which setting speed to 0 does not work
        // You must set a speed slightly away from zero
        if (speed < 0.001 && speed > -0.001) {
            speed = 0.005;
        }
        if (lastSpeed == speed)
            return;
        motor.set(speed);
        lastSpeed = speed;
    }

    public void setSpeedAbsolute(double speed) {
        // Make up for bug in Rev motor in which setting speed to 0 does not work
        // You must set a speed slightly away from zero
        if (speed < 0.001 && speed > -0.001) {
            motor.stopMotor();
            lastSpeed = 0;
        } else {
            motor.set(speed);
            lastSpeed = speed;
        }
    }

    public void setVelocity(double velocity) {
        motor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void stopMotor() {
        motor.stopMotor();
        lastSpeed = 0;
    }

    public void setVolts(double volts) {
        motor.setVoltage(0);
    }

    public double getMotorVoltage() {
        return motor.getAppliedOutput();
    }

    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    public int getPos() {
        return (int) (relEncoder.getPosition());
    }

    public void zeroEncoder() {
        relEncoder.setPosition(0);
    }

    public void setEncoderPosition(double position) {
        relEncoder.setPosition(position);
    }

    public void setPos(double position) {
        this.desiredPositionTicks = position;
        logf("-------- Set motor %s at %.1f ticks\n", name, desiredPositionTicks);
        relEncoder.setPosition(position);
    }

    public boolean getForwardLimitSwitch() {
        return forwardSwitch.isPressed();
    }

    public boolean getReverseLimitSwitch() {
        return reverseSwitch.isPressed();
    }

    public void setActualPosition(double position) {
        motor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
        Util.logf("Set Actual Name:%s Pos:%.1f P:%.6f I:%.6f D:%.6f\n", name, position, motor.getPIDController().getP(),
                motor.getPIDController().getI(), motor.getPIDController().getD());
        // m_pidController.setReference(rotations, ControlType.kPosition);
    }

    // Rate time in seconds to go from idle to full speed
    public void setRampOpenLoop(double rate) {
        motor.setOpenLoopRampRate(rate);
    };

    public void setRampClosedLoop(double rate) {
        motor.setOpenLoopRampRate(rate);
    };

    SparkMaxPIDController getPIDController() {
        return motor.getPIDController();
    }

    public void logPeriodic() {
        int pos = getPos();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logf(getMotorVCS(motor) + getMotorVCS(followMotor));
                } else {
                    logf(getMotorVCS(motor));
                }
            }
        }
    }

    public void updateSmart() {
        SmartDashboard.putNumber(name + " Pos", getPos());
        SmartDashboard.putNumber(name + " Cur", round2(motor.getOutputCurrent()));
    }

    public void periodic() {
        if (!Robot.config.showMotorData)
            return;
        switch ((int) count % 12) {
            case 0:
                periodicHit++;
                if (followId > 0)
                    currentRear = followMotor.getOutputCurrent();
                break;
            case 1:
                if (followId > 0)
                    temperatureRear = followMotor.getMotorTemperature();
                break;
            case 2:
                if (followId > 0)
                    velocityRear = relEncoderFollow.getVelocity();
                break;
            case 3:
                if (followId > 0)
                    encoderRear = (long) (relEncoder.getPosition());
                break;
            case 4:
                current = motor.getOutputCurrent();
                break;
            case 5:
                temperature = motor.getMotorTemperature();
                break;
            case 6:
                velocity = relEncoder.getVelocity();
                break;
            case 7:
                encoder = (long) relEncoder.getPosition();
                if (encoder == lastEncoder) {
                    encoderHit = false;
                } else {
                    encoderHit = true;
                    lastEncoder = encoder;
                }
                break;
            case 8:
                if (periodicHit % 4 == 0 && myLogging && encoderHit)
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%d\n", name, lastSpeed, current, temperature,
                            velocity, encoder);
                break;
            case 9:
                if (followId > 0 && periodicHit % 4 == 2 && myLogging && encoderHit)
                    logf("%s rear cur:%.2f temp:%.2f vel:%.2f pos:%d\n", name, currentRear, temperatureRear,
                            velocityRear, encoderRear);
                break;
            case 10:
                SmartDashboard.putNumber(name + " Pos", round2(encoder));
                break;
            case 11:
                SmartDashboard.putNumber(name + " Cur", round2(current));
                break;
        }
    }

    void testTimes() {
        long startTime = RobotController.getFPGATime();
        double sp = motor.get();
        long endTime1 = RobotController.getFPGATime();
        double cur = motor.getOutputCurrent();
        long endTime2 = RobotController.getFPGATime();
        double enc = relEncoder.getPosition();
        long endTime3 = RobotController.getFPGATime();
        logf("%s motor sp:%.2f pos:%.0f\n", name, sp, cur, enc);
        long endTime = RobotController.getFPGATime();
        System.out.printf("get:%d cur:%d enc:%d log:%d total:%d\n", endTime1 - startTime, endTime2 - endTime1,
                endTime3 - endTime2, endTime - endTime3, endTime - startTime);
    }

    public void logPeriodicSlow() {
        if (followId > 0) {
            logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, motor.get(), motor.getOutputCurrent(),
                    motor.getMotorTemperature(), relEncoder.getVelocity(), relEncoder.getPosition());
            logf("%s rear  sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, followMotor.get(),
                    followMotor.getOutputCurrent(), followMotor.getMotorTemperature(), relEncoderFollow.getVelocity(),
                    relEncoderFollow.getPosition());
        } else {
            logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, motor.get(), motor.getOutputCurrent(),
                    motor.getMotorTemperature(), relEncoder.getVelocity(), relEncoder.getPosition());
        }
    }

    public String getMotorVCS() {
        return getMotorVCS(motor);
    }

    private String getMotorVCS(CANSparkMax motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputCurrent = motor.getOutputCurrent();
            return String.format("%s motor volts:%.2f cur:%.2f power:%.2f sp:%.3f", name, bussVoltage,
                    outputCurrent, bussVoltage * outputCurrent, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getLastSpeed() {
        return lastSpeed;
    }

    public void logMotorVCS() {
        // if (Math.abs(motor.get()) < .01)
        // logf("%s motor not Running\n", name);
        if (Math.abs(motor.get()) > .01) {
            if (followId > 0) {
                logf("%s motor volts<%.2f,%.2f> cur<%.2f,%.2f> sp<%.2f,%.2f>\n", name, motor.getBusVoltage(),
                        followMotor.getBusVoltage(), motor.getOutputCurrent(), followMotor.getOutputCurrent(),
                        motor.get(), followMotor.get());
            } else {
                logf("%s motor volts:%.2f cur:%.2f sp:%.2f\n", name, motor.getBusVoltage(), motor.getOutputCurrent(),
                        motor.get());
            }
        }
    }

    public void logAllMotor() {
        if (followId > 0) {
            logf("%s,bv,%.2f,%.2f,av,%.2f,%.2f,oc,%.2f,%.2f,sp,%.2f,%.2f,enc,%.0f,%.0f,vel,%.3f,%.3f\n", name,
                    motor.getBusVoltage(), followMotor.getBusVoltage(), motor.getAppliedOutput(),
                    followMotor.getAppliedOutput(), motor.getOutputCurrent(), followMotor.getOutputCurrent(),
                    motor.get(), followMotor.get(), relEncoder.getPosition(), relEncoderFollow.getPosition(),
                    relEncoder.getVelocity(), relEncoderFollow.getVelocity());
        } else {
            logf("%s motor volts:%.2f cur:%.2f sp:%.2f\n", name, motor.getBusVoltage(), motor.getOutputCurrent(),
                    motor.get());
        }
    }

    public void PIDToMotor(PID pid, int slot, int timeOut) {
        logf("Setup %s REV PID %s\n", name, pid.getPidData());
        SparkMaxPIDController ctrl = motor.getPIDController();
        // set PID coefficients
        ctrl.setP(pid.kP, slot);
        ctrl.setI(pid.kI, slot);
        ctrl.setD(pid.kD, slot);
        ctrl.setIZone(pid.kIz, slot);
        ctrl.setFF(pid.kFF, slot);
        ctrl.setOutputRange(pid.kMinOutput, pid.kMaxOutput, slot);
    }
}