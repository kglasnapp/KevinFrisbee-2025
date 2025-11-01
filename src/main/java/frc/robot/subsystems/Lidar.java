package frc.robot.subsystems;

import java.util.TimerTask;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utilities.Util;

public class Lidar extends SubsystemBase {
    private I2C i2c;
    private byte[] distance;
    private double distInches = 0.0;
    private double distFeet = 0.0;
    private java.util.Timer updater;
    private final int LIDAR_ADDR = 0x62;
    private final int LIDAR_CONFIG_REGISTER = 0x00;
    private int lidarErrors = 0;
    private MySolenoidPCM light;
    private double lastDist;
    private boolean lidarValid = false;

    public Lidar() {
        Port port = I2C.Port.kMXP;
        // Port port = I2C.Port.kOnboard;
        i2c = new I2C(port, LIDAR_ADDR);
        // distance = new byte[1];
        distance = new byte[2];
        updater = new java.util.Timer();
        setup();
        Timer.delay(.030);
        light = new MySolenoidPCM("Test",1, 5, -5, false);
        light.setA(true);
        start();
    }

      @Override
    public void periodic() {
        // If Lidar is not in range of field consider it invalid
        lidarValid = distFeet > 1 && distFeet < 35;
        if (Robot.count % 15 == 2 && Robot.config.ShowOnSmart) {
            SmartDashboard.putNumber("Lidar (feet)", Util.round2(distFeet));
            SmartDashboard.putBoolean("LidarValid", lidarValid);

        }
        // blinkForLight(Robot.shooter.getPreparedDistance(), distFeet);

    }

    public double getDistanceInches() {
        return Util.round2(distInches);
    }

    public boolean isLidarValid() {
        return lidarValid;
    }

    // Start polling
    public void start() {
        // Was 40 on 10/12/16, Radu had it at 100
        updater.scheduleAtFixedRate(new LIDARUpdater(), 0, 50);
    }

    private void processError(boolean w1, boolean r1, boolean r2) {
        if ((w1 || r1 || r2)) {
            Util.logf("???? LIDAR Error %d w1:%b r1:%b r2:%b %n", lidarErrors, w1, r1, r2);
            lidarErrors++;
            distFeet = 0;
            distInches = 0;
        }
    }

    // Update distance variable for Lidar V3
    public void update() {
        byte cmd[] = new byte[1];
        i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // With Receiver Bias
        cmd[0] = 0xf;
        i2c.writeBulk(cmd);
        boolean r1 = i2c.readOnly(distance, 1);
        int d0 = Byte.toUnsignedInt(distance[0]);
        cmd[0] = 0x10;
        i2c.writeBulk(cmd);
        boolean r2 = i2c.readOnly(distance, 1);
        int d1 = Byte.toUnsignedInt(distance[0]);
        double distCM = 256 * d0 + d1;
        processError(r2, r1, false);
        distInches = distCM / 2.54;
        distFeet = distInches / 12.0;
        // if (Robot.count % 5 == 4) {
        // if (Math.abs(distFeet - lastReported) > .2) {
        // // Util.logf("Lidar Distance feet:%.2f\n", distFeet);
        // }
        // lastReported = distFeet;

    }

    public void setup() {
        boolean w1 = i2c.write(0x0, 0x0);
        Timer.delay(0.050);
        boolean w2 = i2c.write(0x02, 0x80);
        boolean w3 = i2c.write(0x04, 0x08);
        boolean w4 = i2c.write(0x1c, 0x00);
        Util.logf("Lidar setup w1:%b w2:%b w3:%b w4:%b\n", w1, w2, w3, w4);
    }

    private class LIDARUpdater extends TimerTask {
        public void run() {
            update();
            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    int lastPulse = 0;
    int logCount = 0;

    public void blinkForLight(double target, double distFeet) { // target value: 20, 15, 5
        // light is solid when near target (half foot), blinks fast when when slightly
        // farther (foot), blinks slow when farther (foot and a half)
        logCount += 1;
        int pulse = -1;
        if (target < 0.1) {
            light.setA(false);
            return;
        }
        if (distFeet <= (target + 0.8) && distFeet >= (target - 0.8)) { // smallest distance
            pulse = 0;
        } else if (distFeet <= (target + 1.4) && distFeet >= (target - 1.4)) {
            pulse = 8;
        } else if (distFeet <= (target + 2.2) && distFeet >= (target - 2.2)) {
            pulse = 4;
        }
        if (pulse == 0)
            light.setA(true);
        else if (Robot.count % pulse == 0 && pulse > 0)
            light.setA(!light.getA());
        else if (pulse == -1)
            light.setA(false);
        if (distFeet != lastDist) {
            if ((target > 0 && Math.abs(lastDist - distFeet) > .2) && (logCount % 25 == 0)) {
                Util.logf("************** LIDAR Dist:%.2f pulse:%d light:%b prep:%.2f\n", distFeet, pulse, light.getA(),
                        target);
            }
            lastDist = distFeet;
            lastPulse = pulse;
        }
    }
}