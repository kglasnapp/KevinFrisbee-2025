package frc.robot;

import static frc.robot.utilities.Util.logf;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Config {

    // Default parameters should defaults for the competition Robot

    public enum RobotType {
        MiniSRX, MiniFast, Competition, Sibling, Kevin
    };

    // Type of Robot
    public static RobotType robotType = RobotType.Kevin;

    // Pneumatic Control Modules Parameters
    public int pcmHubID = 1;

    // Vision Parameters
    public boolean LimeLight = false;
    public boolean ledRing = false;

    public enum DriveType {
        None, Tank, OperatorTank,
    };

    // Drive Parameters
    public int driveRight = 2;
    public int driveLeft = 3;
    public int driveRightFollow = 4;
    public int driveLeftFollow = 5;
    public DriveType driveType = DriveType.OperatorTank;
    public boolean invertDrivetrain = false;
    public double driveTicksPerInch = 987;
    public boolean defaultBrakeMode = true;
    public double wheelBase = 15.5; // Wheel base for mini
    public double wheelDiameter = 6.0; // Wheel diameter for MINI
    public double driveTicksPerRevolution = 2000; // Value for mini
    public double throttleRate = 1.5; // for the throttle power curve
    public boolean showMotorData = false;
    public double rampUp = .05;
    public double rampUpTurbo = .05;
    public double rampDown = .05;
    public double rampDownTurbo = .05;
    public double speedFactor = .5;
    public double initialPoseX = -45;
    public double initialPoseY = -81;
    public double initalPoseAngle = Math.toDegrees(Math.atan2(initialPoseY, initialPoseX));

    // Climber Parameters
    public boolean climber = true;
    public double climberTicksPerIn = 260 - 10;
    public int rightClimbMotorId = 6;
    public int leftClimbMotorId = 7;
    public double climberP = .6 + 1 + .5 + 2 + 1; // Added .5 3/3/2022 -- added another 1 on 3/15
    public int SiblingClimbMotorID = 20;

    // Miscellaneous Parameter
    public boolean ultraSonicDistance = true;
    public boolean ultra1030Front = false;
    public boolean ultra1030Rear = false;
    public int kTimeoutMs = 30; // default timeout used for messages to the SRX
    public boolean enableCompressor = false;
    public boolean cameraServer = true;
    public boolean joysticksEnabled = true;
    public boolean operatorPadEnabled = true;
    public boolean driverPadEnabled = false;
    public boolean neoPixelsActive = false;
    public double deadZone = 0.085;
    public boolean powerHubToDashBoard = false;
    public int blinkerChannel = 13;
    public boolean ColorSensor = false;
    public PneumaticsModuleType pneumaticType = PneumaticsModuleType.CTREPCM;

    // Shooter parameters
    public boolean shooter = true;
    public boolean shooterVelocityPID = true;
    public int shooterID = 12;
    public double ShooterSpeedLow = .5;
    public double ShooterSpeedMedium = .7;
    public double ShooterSpeedHigh = 1;

    public int ballLiftDelay = 20;
    public double beaterBarSpeed = 0.75; // Changed from .85 used at WPB, 0.75
    public double intakeSpeed = 0.5; // Changed from 0.6 bc wheels pushing up due to speed
    public double intakeSpeedShooting = 0.9;

    // Sibling Magazine (Intake) Parameters
    public double topMagazineSpeed = .8;
    public double bottomMagazineSpeed = .8;
    public double siblingBeaterBarSpeed = .75;

    // Misc parameters
    public boolean lidar = false;
    public boolean pigeon = false;
    public boolean BNO055Connected = false;
    public boolean PowerDistributionHub = true;
    public boolean PneumaticHUB = false;
    public boolean PhotonVision = true;
    public boolean ShowOnSmart = false;
    public boolean BlinkTarget = false;
    public boolean UDP = false;

    // Shoot Command parameters
    public double conveyorSpeed = 0.7;
    public double highShootSpeedTop = 1.0;
    public double highShootSpeedBottom = 0.6;
    public double lowShootSpeedTop = 0.3;
    public double lowShootSpeedBottom = 0.3;
    public final double CERTAIN_SPEED = 0.6;
    public final boolean Vision = false;

    Config() {
        if (isMini()) {
            robotType = RobotType.MiniFast;
            robotType = RobotType.Kevin;
        }
        logf("Start of Robot Config for %s\n", robotType);
        switch (robotType) {
            case Kevin:
                pneumaticType = PneumaticsModuleType.CTREPCM;
                invertDrivetrain = true;
                
                shooterID = 12;
                ultraSonicDistance = false;
                cameraServer = false;
                initialPoseX = 0;
                initialPoseY = 0;
                break;
            case MiniSRX:
                joysticksEnabled = false;
                break;
            case MiniFast:
                driveRight = 2;
                driveLeft = 3;
                driveRightFollow = -7;
                driveLeftFollow = -11;
                driveTicksPerInch = (50 * 12) / 16;
                climberP = .1;
                climber = false;
                PowerDistributionHub = false;
                shooter = false;
                cameraServer = false;
                enableCompressor = false;
                PhotonVision = true;
                BlinkTarget = false;
                ultra1030Front = true;
                SiblingClimbMotorID = -20;
                PneumaticHUB = false;
                joysticksEnabled = false;
                speedFactor = .5;
                break;
            case Competition:
                ultra1030Front = true;
                beaterBarSpeed = 0.75;
                BlinkTarget = true;
                break;
            case Sibling:
                cameraServer = true;
                if (isMiniSibling()) {
                    logf("Set parms for Mini Sibling\n");
                    driveRightFollow = -4;
                    driveLeftFollow = -5;
                    PowerDistributionHub = false;
                    enableCompressor = false;
                    shooterID = -12;

                }
                joysticksEnabled = true;
                driverPadEnabled = true;
                climber = false;
                ultra1030Front = true;
                BlinkTarget = true;
                break;
        }
    }

    public RobotType getRobotType() {
        return robotType;
    }

    public boolean isMini() {
        String fileName = "/home/lvuser/deploy/mini.txt";
        File fin = new File(fileName);
        return fin.exists();
    }

    public boolean isMiniSibling() {
        String fileName = "/home/lvuser/deploy/miniSibling.txt";
        File fin = new File(fileName);
        return fin.exists();
    }

    void readConfig() {
        String fileName = "/home/lvuser/deploy/parameters.txt";
        File fin = new File(fileName);
        if (!fin.exists()) {
            logf("File %s not found default values assumed", fileName);
            return;
        }
        try {
            BufferedReader br = new BufferedReader(new FileReader(fin));
            String line = null;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.startsWith("#")) {
                    String[] ar = line.split("=");
                    if (ar.length >= 2) {
                        String value = ar[1].trim();
                        if (value.contains("#")) {
                            value = value.split("#")[0].trim();
                        }
                        setVariable(ar[0].trim(), value);
                    }
                }
            }
            br.close();
        } catch (FileNotFoundException ex) {
            logf("Unable to open file: %s\n", fin.getName());
        } catch (IOException ex) {
            logf("Error reading file: %s\n", fin.getName());
        }
    }

    boolean setVariable(String variable, String value) {
        try {
            switch (variable.toLowerCase()) {
                case "robottype":
                    robotType = RobotType.valueOf(value);
                    break;
                default:
                    logf("???? Warning Variable:%s not found in parms.txt file\n", variable);
                    return false;
            }
        } catch (Exception ex) {
            logf("Unable to convert %s with a value of %s\n", variable, value);
            return false;
        }
        logf("Variable:%s value:%s\n", variable, value);
        return true;
    }
}