package frc.robot.utilities;

public class RunningAverage {
    private int lastSample = 0;
    private int numSamples = 0;
    private final double[] samples;
    private int size = 0;

    public RunningAverage(int size) {
        samples = new double[size];
        this.size = size;
    }

    public void init() {
        for(int i = 0; i < size; i++)
          samples[i] = 0;
        numSamples = 0;
    }

    public double add(double d) {
        if (lastSample >= samples.length)
            lastSample = 0;
        samples[lastSample] = d;
        lastSample++;
        if (numSamples != samples.length)
            numSamples++;
        return getAverage();
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
