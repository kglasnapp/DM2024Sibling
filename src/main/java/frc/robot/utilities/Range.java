package frc.robot.utilities;

public class Range {
    public static final Range ALL_REALS = new Range(Double.MIN_VALUE, Double.MAX_VALUE);

    public final double min, max;
    public final double range;

    /**
     * @throws IllegalArgumentException if minimum greater than maximum.
     */
    public Range(double min, double max) {
        if (min > max)
            throw new IllegalArgumentException("Minimum cannot be greater than maximum.");
        this.min = min;
        this.max = max;
        this.range = max - min;
    }

    public boolean inRange(double value) {
        return value >= min && value <= max;
    }

    /**
     * @param value Value to clamp.
     * @return Either less than or equal to max or greater than or equal to min.
     */
    public double clamp(double value) {
        return clamp(value, min, max);
    }

    @Override
    public String toString() {
        return "Range[" + min + ", " + max + "]";
    }

    /**
     * Utility clamp function. If the min boundary is equal to the max, the value is returned.
     *
     * @param value Value to clamp.
     * @param min Min bound
     * @param max Max bound
     * @return Either less than or equal to max or greater than or equal to min.
     */
    public static double clamp(double value, double min, double max) {
        if (min == max)
            return value;
        if (value >= max)
            return max;
        else if (value <= min)
            return min;
        return value;
    }

    public static double clamp(double value, double range) {
        return Math.copySign(Math.min(Math.copySign(value, 1), range), value);
    }
}