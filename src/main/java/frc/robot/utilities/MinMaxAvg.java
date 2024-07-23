package frc.robot.utilities;

public class MinMaxAvg {
    int count = 0;
    double min = 1e10;
    double max = 0;
    double sum = 0;

    public void AddData(double data) {
        max = Math.max(max, data);
        min = Math.min(min, data);
        sum += data;
    }

    public String Show(boolean zero) {
        String s = String.format("Avg:%.2f Min:%.2f Max:%.2f", sum / count, min, max);
        if (zero) {
            count = 0;
            min = 1e10;
            max = 0;
            sum = 0;
        }
        return s;
    }
}
