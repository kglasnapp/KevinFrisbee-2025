// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;
import static frc.robot.utilities.Util.splashScreen;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Config.RobotType;
import frc.robot.commands.SetDriveMode;
import frc.robot.commands.groups.Test;
import frc.robot.subsystems.CompressorController;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.KevinShooter;
import frc.robot.subsystems.VisionData;
import frc.robot.subsystems.VisionLight;
import frc.robot.subsystems.YawProvider;
import frc.robot.utilities.MinMaxAvg;
import frc.robot.subsystems.Lidar;

/**
 * Things to do
 * 
 */

public class Robot extends TimedRobot {
  public static Config config = new Config();
  public static Drivetrain drivetrain;
  public static long count = 1;
  public static boolean logging = false;

  private String version = "2.0";
  public static double frontBallVoltage = 0;
  public static double rearBallVoltage = 0;
  public static double frontDistance = 0;
  public static double rearDistance = 0;
  public static double yaw; // Robots yaw as determined by the active Yaw sensor
  private static long lastMemory;
  public static Joysticks joysticks = new Joysticks();
  public static OI oi;
  public static YawProvider yawNavX;
  private long lastGC = 0;
  private MinMaxAvg loopData = new MinMaxAvg();
  private MinMaxAvg gcData = new MinMaxAvg();

  public static KevinShooter shooter;
  public static CompressorController compressor;
  public static Sensors distanceSensors;
  public static long longestLoopTime = 0;
  public static PneumaticHub pHub;
  // Change to another type if using a different vision solution
  public static VisionData vision;
  public static VisionLight visionLight;
  public static Lidar lidar;
  public static Alliance alliance;
  public static int location;
  public static DriveMode driveMode = DriveMode.OPERATOR_MILD;
  public static boolean driveJoy = false;
  public static boolean driveArcade = true;
  PowerDistribution PDH = new PowerDistribution();

  public enum DriveMode {
    NOT_ASSIGNED, JOY_AGRESSIVE, JOY_MILD, OPERATOR_AGRESSIVE, OPERATOR_MILD, ARCADE_MILD, ARCADE_AGRESSIVE
  };

  // A chooser for autonomous commands
  SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  // A chooser for drive mode
  SendableChooser<DriveMode> driveChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // alliance = DriverStation.getAlliance();
    // location = DriverStation.getLocation();
    // logf("Alliance:%s Location:%d Free Mem at start %d\n", alliance.toString(), location, getMemory());
    splashScreen(version);
    drivetrain = new Drivetrain();
    drivetrain.setBrakeMode(false);
    oi = new OI();
    yawNavX = new YawProvider();
    if (Robot.config.PneumaticHUB)
      pHub = new PneumaticHub();

    if (config.shooter) {
      shooter = new KevinShooter();
    }
    if (config.ultraSonicDistance) {
      distanceSensors = new Sensors();
    }
    if (config.enableCompressor) {
      compressor = new CompressorController();
    }

    if (config.cameraServer) {
      CameraServer.startAutomaticCapture();
    }

   

    if (config.BlinkTarget)
      visionLight = new VisionLight();

    // Add commands to the autonomous command chooser
    autonomousChooser.addOption("Test", new Test());
    // Put the chooser on the dashboard
    SmartDashboard.putData("Autonomous Mode", (Sendable) autonomousChooser);

    // Add command for the Drive Mode chooser
    driveChooser.setDefaultOption("Game Pad Mild", DriveMode.OPERATOR_MILD);
    driveChooser.addOption("Dual Joystick Aggressive", DriveMode.JOY_AGRESSIVE);
    driveChooser.addOption("Game Pad Aggressive", DriveMode.OPERATOR_AGRESSIVE);
    driveChooser.addOption("Dual Joystick Mild", DriveMode.JOY_MILD);
    driveChooser.addOption("Arcade Mode Mild", DriveMode.ARCADE_MILD);
    driveChooser.addOption("Arcade Mode Aggressive", DriveMode.ARCADE_MILD);
    // Put the chooser on the dashboard
    SmartDashboard.putData("Drive Type", (Sendable) driveChooser);

    // Zero YAW at init
    yawNavX.zeroYaw();

  

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    // Note loop stat time

    long loopStart = RobotController.getFPGATime();
    // Get the current robot yaw
    yaw = yawNavX.yaw();
    // Put Yaw on smart dashbaord every 0.5 seconds
    if (count % 25 == 16) {
      SmartDashboard.putNumber("Yaw", round2(yaw));
    }
    if (count == 10 && vision != null) {
      vision.cameraLight(false);
    }
    if (Robot.count % 5 == 0 && distanceSensors != null) {
      frontDistance = distanceSensors.getFrontDistance();
      rearDistance = distanceSensors.getRearDistance();
    }
    if (Robot.count % 20 == 10 && vision != null) {
      SmartDashboard.putBoolean("VV", vision.getV());
    }

    // Code for the mini drive station
    int disableCount = (int) SmartDashboard.getNumber("Disable", -1);
    if (disableCount == 3932) {
      logf("!!!!!!!!!!!!!! Will Disable the robot from mini drive station !!!!!!!!!!!!!\n");
      // Set Bat Volts to 0 to indicate to the mini drive station that the robot is
      // disabled
      SmartDashboard.putNumber("BatVolts", 0);
      System.exit(0); // Disable the robot
    }

    if (count % 100 == 0) {
      boolean dis = isDisabled();
      if (PDH != null) {
        SmartDashboard.putNumber("BatVolts",  (!dis) ? round2(PDH.getVoltage()) + Math.random() * .1 : -Math.random() * .1 );
      } else {
        SmartDashboard.putNumber("BatVolts", (!dis) ? 12.0 + Math.random() * .1 : 0 - Math.random() * .1);
      }
    }
  

    // Setup the drive mode if not set
    if (count == 20) {
      logf("***** Drive Chosser at start %s\n", driveChooser.getSelected());
      Command cmd = new SetDriveMode(driveChooser.getSelected());
      cmd.schedule();
    }
    if (count % 25 == 8) {
      DriveMode mode = driveChooser.getSelected();
      if (mode != driveMode) {
        logf("***** Drive mode changed old:%s new:%s\n", driveMode, driveChooser.getSelected());
        if (mode == DriveMode.JOY_AGRESSIVE || mode == DriveMode.JOY_MILD) {
          if (!joysticks.getJoysPresent()) {
            logf("!!!!!!!!!!!  No joysticks so can't change to new mode\n");
          }
        } else {
          Command cmd = new SetDriveMode(mode);
          cmd.schedule();
        }
      }
    }

    // Run the tasks
    CommandScheduler.getInstance().run();

    // Show longest loop time over 15 millisecond every second
    long loopTime = RobotController.getFPGATime() - loopStart;
    longestLoopTime = Math.max(longestLoopTime, loopTime);
    // Every second reset max loop time
    if (count % 50 == 16) {
      // Show loop time in milli-seconds
      loopData.AddData(longestLoopTime / 1000.0);
      longestLoopTime = 0;
    }

    // Determine GC Rate
    long mem = getMemory();
    if (mem > lastMemory) {
      gcData.AddData((count - lastGC) * .02);
      lastGC = count;
    }
    lastMemory = mem;

    // Log GC and Max loop data every minute
    if (count % (50 * 60) == 210) {
      logf("Loop Count %s Garbage Collector %s\n", loopData.Show(true), gcData.Show(true));
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    logf("Robot was disabled\n");

    // Set Bat Volts to 0 to indicate to the mini drive station that the robot is
    // disabled
    SmartDashboard.putNumber("BatVolts", 0);

    if (shooter != null) {
      shooter.stopShooter(); // stop shooter from spinning on enable if spinning during disable
    }
    if (shooter != null) {
      shooter.stopShooter(); // stop shooter from spinning on enable if spinning during disable
    }
    drivetrain.setBrakeMode(false);
  }

  @Override
  public void disabledPeriodic() {
    count += 1;
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    drivetrain.setBrakeMode(true);
    Command cmd = autonomousChooser.getSelected();
    drivetrain.resetEncoders();
    count = 0;
    // schedule the autonomous command
    if (autonomousChooser != null) {
      cmd.schedule();
    }
  }

  @Override
  public void teleopInit() {
    logf("Start of Teleop\n");
    count = 0;
    drivetrain.setBrakeMode(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    count += 1; // Main counter used to time things in the robot
  }

  @Override
  public void autonomousPeriodic() {

    count += 1; // Main counter used to time things in the robot
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public long getMemory() {
    return Runtime.getRuntime().freeMemory();
  }

}
