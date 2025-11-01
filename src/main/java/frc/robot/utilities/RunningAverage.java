package frc.robot.utilities;

public class RunningAverage {
    private int lastSample = 0;
    private int numSamples = 0;
    private final double[] samples;

    public RunningAverage(int size) {
        samples = new double[size];
    }

    public void add(double d) {
        if (lastSample >= samples.length)
            lastSample = 0;
        samples[lastSample] = d;
        lastSample++;
        if (numSamples != samples.length)
            numSamples++;
    }

    public double getAverage() {
        double sum = 0;
        for (int i = 0; i < numSamples; i++)
            sum += samples[i];
        return sum / numSamples;
    }

    public double deviation() {
        double deviation = 0;
        double avg = getAverage();
        for (int i = 0; i < numSamples; i++)
            deviation += Math.abs(samples[i] - avg);
        return deviation / numSamples;
    }

    public boolean isAverageValid() {
        return numSamples == samples.length;
    }
}
