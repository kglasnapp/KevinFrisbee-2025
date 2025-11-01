package frc.robot.subsystems;

public interface VisionData {
    public double getX();

    public double getY();

    public boolean getV();

    public double getArea();

    public void cameraLight(boolean state);

    public void takeSnapshot();
}
