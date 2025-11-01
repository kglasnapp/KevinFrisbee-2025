package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.utilities.Util.logf;

public class KevinShooter extends SubsystemBase {
  private MotorSRX shooterMotor;
  private double lastSpeed = 0;
  private int lastPOV = -1;
  private MySolenoidPCM pushFrisbee;
  private MySolenoidPCM dropFrisbee;
  private MySolenoidPCM liftTop;
  private MySolenoidPCM liftArm;
  private boolean armsUp = false;

  // Current threshold to trigger current limit
  private int kPeakCurrentAmps = 60;
  // Duration after current exceed Peak Current to trigger current limit
  private int kPeakTimeMs = 0;
  // Current to mantain once current limit has been triggered
  private int kContinCurrentAmps = 40;

  public KevinShooter() {
    shooterMotor = new MotorSRX("Shooter", Robot.config.shooterID, -1, true);
    shooterMotor.setBrakeMode(false);
    shooterMotor.setCurrentLimit(kPeakCurrentAmps, kContinCurrentAmps, kPeakTimeMs);

    
    // 2 and 3 is push, 4 and 5 are drop,  0 nd 1 is arm, 6 and 7  top
    // Define the solenoids for the pneumatics
    pushFrisbee = new MySolenoidPCM("Pusher", 1, 3, 2, true);
    dropFrisbee = new MySolenoidPCM("Drop", 1, 4, 5, true);
    
  
    dropFrisbee.setA(true); // makes it so the pusher is out once we start the robot
    
    liftTop = new MySolenoidPCM("Top", 1, 6, 7, true);
    liftArm = new MySolenoidPCM("Arm", 1, 0, 1, true);
    //arms are 0 and 1
    logf("Kevin shooter is setup at ID:%d\n", Robot.config.shooterID);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    int operatorPOV = Joysticks.operator.getPOV();
    // Set shooter speed based upon operator POV
    if (operatorPOV >= 0 && Robot.config.shooterVelocityPID && lastPOV != operatorPOV) {
      double[] shootSpeed = { 0, Robot.config.ShooterSpeedLow, Robot.config.ShooterSpeedMedium,
          Robot.config.ShooterSpeedHigh };
      double newVelocity = shootSpeed[operatorPOV / 90];
      setShooterSpeed(newVelocity);
    }
    if (Robot.count % 500 == 200 && getShooterSpeed() > 0) {
      logf("Shooter speed:%.2f req:%.2f\n", getShooterSpeed(), lastSpeed);
    }
    if (Robot.count % 15 == 6) {
      SmartDashboard.putNumber("Sh Current", getMotorCurrent());
    }
  }

  public void setShooterSpeed(double speed) {
    if (lastSpeed == speed) {
      return;
    }
    logf("New Shooter speed:%.2f last:%.2f\n", speed, lastSpeed);
    shooterMotor.setSpeed(speed);
    lastSpeed = speed;
  }

  public double getShooterSpeed() {
    return shooterMotor.getActualVelocity();
  }

  public double getMotorCurrent() {
    return shooterMotor.getMotorCurrent();
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  public void shooterReverse() {
    logf("Start shooter reverse\n");
    shooterMotor.setSpeed(-0.5); // arbitrary rpm to spit out Fribee if stuck
  }

  public void activatePusher() {
    pushFrisbee.pulseA();
  }

  public void releasePusher() {
    pushFrisbee.pulseB();
  }

  public void activateDroper() {
    dropFrisbee.setA(true);
  }

  public void releaseDroper() {
    dropFrisbee.setA(false);
  }

  public void activateTop(){
    liftTop.pulseA();
  }

  public void releaseTop(){
    liftTop.pulseB();
  }
  public void toggleArms(){
    if (armsUp) 
      {
        liftArm.pulseB();
      }
      else{
        liftArm.pulseA();
      }
      armsUp = !armsUp; 

  }
}